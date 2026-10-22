//! A scenario decoded from bytes, run on one model through the real bootloader and updater, checked after every
//! boot against what must hold whatever the power did.

use core::cell::RefCell;
use std::fmt::Write as _;

use embassy_boot::{
    BlockingFirmwareState, BlockingFirmwareUpdater, BootLoader, BootLoaderConfig, FirmwareUpdaterConfig, State,
};
use embassy_embedded_hal::flash::partition::BlockingPartition;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;

use super::flash::{Model, Nor, Regions, Rng, SimError};
use super::zoo::{self, Expect, Visit};

/// Reads numbers from the input; zeros once it runs out.
pub struct Input<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Input<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Self { data, pos: 0 }
    }

    pub fn done(&self) -> bool {
        self.pos >= self.data.len()
    }

    pub fn byte(&mut self) -> u8 {
        let b = self.data.get(self.pos).copied().unwrap_or(0);
        self.pos += 1;
        b
    }

    pub fn below(&mut self, n: usize) -> usize {
        self.byte() as usize % n
    }

    pub fn u16(&mut self) -> u16 {
        u16::from_le_bytes([self.byte(), self.byte()])
    }
}

/// One thing that happens to the device.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    /// The application downloads a new image into DFU with `write_firmware`, in chunks of a size picked by `chunk`.
    Download { chunk: u8 },
    /// `mark_updated`.
    MarkUpdated,
    /// A reset: the bootloader runs `prepare_boot`, then the application starts.
    Boot,
    /// `mark_booted`.
    MarkBooted,
    /// `mark_dfu`.
    MarkDfu,
    /// Power is cut during the program or erase `after` operations from now; `tear` picks what it leaves.
    Cut { after: u16, tear: u8 },
}

impl Step {
    fn decode(input: &mut Input) -> Step {
        match input.below(6) {
            0 => Step::Download { chunk: input.byte() },
            1 => Step::MarkUpdated,
            2 => Step::Boot,
            3 => Step::MarkBooted,
            4 => Step::MarkDfu,
            _ => Step::Cut {
                after: input.u16(),
                tear: input.byte(),
            },
        }
    }

    pub fn encode(self, out: &mut Vec<u8>) {
        match self {
            Step::Download { chunk } => out.extend([0, chunk]),
            Step::MarkUpdated => out.push(1),
            Step::Boot => out.push(2),
            Step::MarkBooted => out.push(3),
            Step::MarkDfu => out.push(4),
            Step::Cut { after, tear } => {
                out.push(5);
                out.extend(after.to_le_bytes());
                out.push(tear);
            }
        }
    }
}

/// The partition layout, in erase units of the model.
#[derive(Clone, Copy, Debug)]
pub struct Layout {
    pub active: usize,
    pub dfu: usize,
    pub state: usize,
    /// `aligned_buf` of the bootloader, in bytes.
    pub buf: usize,
}

impl Layout {
    /// Bytes of the layout header: active pages, extra DFU pages, extra STATE pages, buffer size.
    pub fn encode(active: u8, dfu_extra: u8, state_extra: u8, buf: u8) -> [u8; 4] {
        [active, dfu_extra, state_extra, buf]
    }

    fn decode<M: Model>(input: &mut Input) -> Layout {
        let active = 1 + input.below(3);
        let dfu = active + 1 + input.below(2);
        // Magic, validity, 2N swap and 2N revert progress words, and a spare.
        let state_bytes = (3 + 4 * active) * M::WRITE_SIZE;
        let state = state_bytes.div_ceil(M::ERASE_SIZE) + input.below(2);
        let bufs = [M::ERASE_SIZE, M::ERASE_SIZE / 2, M::ERASE_SIZE / 8];
        let buf = bufs[input.below(3)].max(M::WRITE_SIZE);
        Layout {
            active,
            dfu,
            state,
            buf,
        }
    }
}

/// An invariant that did not hold.
#[derive(Clone, Debug)]
pub struct Failure {
    pub invariant: &'static str,
    pub detail: String,
    /// A known class, named, so that a sweep can tell it from a new one.
    pub known: Option<&'static str>,
}

/// What a run did.
pub struct Outcome {
    pub expect: Expect,
    pub failure: Option<Failure>,
    pub ops: u64,
    pub report: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Tri {
    No,
    Maybe,
    Yes,
}

/// What must be true, kept beside the flash.
struct Truth {
    /// The confirmed image.
    a: u8,
    /// A complete image in DFU, not yet running.
    b: Option<u8>,
    /// `mark_updated` returned Ok (Yes) or was cut (Maybe).
    staged: Tri,
    /// The bootloader swapped `b` in and the application runs it unconfirmed.
    trial: bool,
    /// During a trial: `mark_booted`/`mark_dfu` was cut.
    confirm: Tri,
    next: u8,
}

/// Content of image `id` at byte `offset`.
fn image_byte(id: u8, offset: usize) -> u8 {
    Rng(((id as u64) << 32) ^ offset as u64).next() as u8
}

fn image_name(id: u8) -> char {
    (b'A' + id - 1) as char
}

struct Sim<M: Model> {
    flash: Mutex<NoopRawMutex, RefCell<Nor<M>>>,
    layout: Layout,
    page: usize,
    state_at: usize,
    active_at: usize,
    dfu_at: usize,
    truth: Truth,
    boots: usize,
}

type Part<'a, M> = BlockingPartition<'a, NoopRawMutex, Nor<M>>;

impl<M: Model> Sim<M> {
    fn new(layout: Layout) -> Self {
        let page = M::ERASE_SIZE;
        let state_at = 0;
        let active_at = layout.state * page;
        let dfu_at = active_at + layout.active * page;
        let size = dfu_at + layout.dfu * page;
        let regions = Regions(vec![
            ("STATE", state_at, layout.state * page),
            ("ACTIVE", active_at, layout.active * page),
            ("DFU", dfu_at, layout.dfu * page),
        ]);
        let mut nor = Nor::<M>::new(size, regions);
        // Image A, as a programmer leaves it.
        for i in 0..layout.active * page {
            nor.mem[active_at + i] = image_byte(1, i);
        }
        Self {
            flash: Mutex::new(RefCell::new(nor)),
            layout,
            page,
            state_at,
            active_at,
            dfu_at,
            truth: Truth {
                a: 1,
                b: None,
                staged: Tri::No,
                trial: false,
                confirm: Tri::No,
                next: 2,
            },
            boots: 0,
        }
    }

    fn nor<R>(&self, f: impl FnOnce(&mut Nor<M>) -> R) -> R {
        self.flash.lock(|cell| f(&mut cell.borrow_mut()))
    }

    fn note(&self, line: String) {
        self.nor(|n| n.note(line));
    }

    fn part(&self, at: usize, len: usize) -> Part<'_, M> {
        BlockingPartition::new(&self.flash, at as u32, len as u32)
    }

    fn state(&self) -> Part<'_, M> {
        self.part(self.state_at, self.layout.state * self.page)
    }

    fn active(&self) -> Part<'_, M> {
        self.part(self.active_at, self.layout.active * self.page)
    }

    fn dfu(&self) -> Part<'_, M> {
        self.part(self.dfu_at, self.layout.dfu * self.page)
    }

    fn dead(&self) -> bool {
        self.nor(|n| n.dead)
    }

    /// An error with no cut behind it: the device is stuck. ECC on STATE is a class of its own.
    fn stuck(&self, what: String) -> Failure {
        let last = self.nor(|n| n.last_error);
        let in_state = last.is_some_and(|(_, at)| self.nor(|n| n.regions.contains("STATE", at)));
        let known = match last {
            Some((SimError::Ecc, _)) if in_state => Some("ecc-state-unreadable"),
            _ => None,
        };
        let cause = match last {
            Some((e, at)) => format!("; the flash said {e:?} at {}", self.nor(|n| n.regions.name(at))),
            None => String::new(),
        };
        Failure {
            invariant: "I1 no power cut, no error",
            detail: format!("{what}{cause}"),
            known,
        }
    }

    /// Which image page `p` of the partition at `at` holds: "B1", "--" erased, "!!" faulting, "??" neither.
    fn page_name(&self, at: usize) -> String {
        self.nor(|n| {
            let mem = &n.mem[at..at + self.page];
            if (at / M::UNIT..(at + self.page) / M::UNIT).any(|u| n.bad[u]) {
                return "!!".into();
            }
            if mem.iter().all(|&b| b == M::ERASED) {
                return "--".into();
            }
            for id in 1..self.truth.next {
                for p in 0..self.layout.active {
                    if mem
                        .iter()
                        .enumerate()
                        .all(|(i, &b)| b == image_byte(id, p * self.page + i))
                    {
                        return format!("{}{p}", image_name(id));
                    }
                }
            }
            "??".into()
        })
    }

    fn pages(&self, at: usize, count: usize) -> Vec<String> {
        (0..count).map(|p| self.page_name(at + p * self.page)).collect()
    }

    fn picture(&self) -> String {
        let words = 2 + 4 * self.layout.active;
        format!(
            "ACTIVE [{}]  DFU [{}]  STATE [{}]",
            self.pages(self.active_at, self.layout.active).join(" "),
            self.pages(self.dfu_at, self.layout.dfu).join(" "),
            self.nor(|n| n.words(self.state_at, words))
        )
    }

    /// The image ACTIVE holds whole, if any.
    fn active_image(&self) -> Option<u8> {
        let pages = self.pages(self.active_at, self.layout.active);
        let first = pages[0].as_bytes();
        let id = first[0].checked_sub(b'A')? + 1;
        let whole = pages
            .iter()
            .enumerate()
            .all(|(p, name)| *name == format!("{}{p}", image_name(id)));
        whole.then_some(id)
    }

    fn boot(&mut self) -> Result<(), Failure> {
        self.nor(|n| n.power_on());
        self.boots += 1;
        self.note(format!("boot {}: prepare_boot", self.boots));
        let mut buf = vec![0u8; self.layout.buf];
        let mut loader = BootLoader::new(BootLoaderConfig {
            active: self.active(),
            dfu: self.dfu(),
            state: self.state(),
        });
        let result = loader.prepare_boot(&mut buf);
        if self.dead() {
            self.note(format!("        -> {result:?} (power cut)    {}", self.picture()));
            // A cut after the swap is done uses up the trial: the next boot reverts without the new image
            // ever running. That is how embassy-boot is meant to work, not a failure.
            if self.truth.staged == Tri::Yes {
                self.truth.staged = Tri::Maybe;
            }
            return Ok(());
        }
        let Ok(_) = result else {
            return Err(self.stuck(format!("prepare_boot returned {result:?}")));
        };
        let mut word = vec![0u8; M::WRITE_SIZE];
        let state = match BlockingFirmwareState::new(self.state(), &mut word).get_state() {
            Ok(state) => state,
            Err(e) => return Err(self.stuck(format!("get_state after the boot returned {e:?}"))),
        };
        self.note(format!(
            "        -> {result:?}, get_state {state:?}    {}",
            self.picture()
        ));
        self.check_boot(state)
    }

    /// The last torn program was a magic other than SWAP into STATE word 0: on byte-wide flash it can read as
    /// SWAP (0xF0 has every 1 bit of 0xE0, 0xD0 and 0xC0), and the bootloader then swaps in what DFU holds.
    fn torn_magic(&self) -> bool {
        self.nor(|n| {
            n.torn
                .as_ref()
                .is_some_and(|(at, bytes)| *at == self.state_at && bytes[0] != 0xF0)
        })
    }

    fn check_boot(&mut self, state: State) -> Result<(), Failure> {
        let known = (state == State::Swap && self.torn_magic()).then_some("torn-magic-reads-swap");
        let fail = |invariant, detail: String| {
            Err(Failure {
                invariant,
                detail,
                known,
            })
        };
        let Some(id) = self.active_image() else {
            return fail(
                "I2 ACTIVE is one whole image",
                format!(
                    "ACTIVE is [{}]",
                    self.pages(self.active_at, self.layout.active).join(" ")
                ),
            );
        };
        let t = &mut self.truth;
        let name = image_name(id);
        match state {
            State::Swap => {
                if Some(id) != t.b {
                    return fail("I4 the state matches ACTIVE", format!("Swap, but ACTIVE is {name}"));
                }
                if t.trial {
                    return fail(
                        "I5 one trial boot",
                        format!("{name} starts a second trial without being confirmed or reverted"),
                    );
                }
                if t.staged == Tri::No {
                    return fail(
                        "I4 the state matches ACTIVE",
                        format!("{name} swapped in, never marked"),
                    );
                }
                t.trial = true;
                t.confirm = Tri::No;
                // The old image must still be there for the revert: DFU pages 1.. hold it.
                let a = image_name(t.a);
                let dfu = self.pages(self.dfu_at + self.page, self.layout.active);
                let want: Vec<String> = (0..self.layout.active).map(|p| format!("{a}{p}")).collect();
                if dfu != want {
                    return fail(
                        "I3 never both images lost",
                        format!(
                            "trial of {name}, but DFU[1..] is [{}], not the old image {a}",
                            dfu.join(" ")
                        ),
                    );
                }
            }
            State::Revert => {
                if Some(id) == t.b && t.trial && t.confirm == Tri::Maybe {
                    // mark_booted/mark_dfu cut after its progress-invalid write: the bootloader takes the
                    // revert path with every copy skipped. The new image stays, the state says Revert.
                    return Err(Failure {
                        invariant: "I4 the state matches ACTIVE",
                        detail: format!("Revert, but ACTIVE is still {name} (the cut was inside the confirm)"),
                        known: Some("confirm-cut-reads-revert"),
                    });
                }
                if id != t.a {
                    return fail("I4 the state matches ACTIVE", format!("Revert, but ACTIVE is {name}"));
                }
                if !t.trial && t.staged == Tri::Yes {
                    return fail(
                        "I4 the state matches ACTIVE",
                        "mark_updated returned Ok, but the bootloader reverted without a trial".into(),
                    );
                }
                t.trial = false;
                t.staged = Tri::No;
                t.confirm = Tri::No;
            }
            State::Boot | State::DfuDetach => {
                if id == t.a {
                    if !t.trial && t.staged == Tri::Yes {
                        return fail(
                            "I4 the state matches ACTIVE",
                            format!("mark_updated returned Ok, but the bootloader booted the old image ({state:?})"),
                        );
                    }
                } else if Some(id) == t.b && t.trial && t.confirm != Tri::No {
                    t.a = id;
                    t.b = None;
                } else {
                    return fail(
                        "I5 one trial boot",
                        format!("{name} runs as confirmed ({state:?}) but was never confirmed"),
                    );
                }
                t.trial = false;
                t.staged = Tri::No;
                t.confirm = Tri::No;
            }
        }
        Ok(())
    }

    /// The application runs: after a cut, the device boots first.
    fn running(&mut self) -> Result<(), Failure> {
        if self.dead() { self.boot() } else { Ok(()) }
    }

    fn step(&mut self, step: Step) -> Result<(), Failure> {
        if !matches!(step, Step::Cut { .. } | Step::Boot) {
            self.running()?;
            if self.dead() {
                return Ok(());
            }
        }
        let mut word = vec![0u8; M::WRITE_SIZE];
        match step {
            Step::Cut { after, tear } => {
                let at = self.nor(|n| n.ops) + after as u64;
                self.nor(|n| n.arm(at, tear as u64 * 0x1_0001 + 7));
                self.note(format!("arm a power cut at op {at} (tear {tear})"));
            }
            Step::Boot => self.boot()?,
            Step::Download { chunk } => {
                let t = &self.truth;
                if t.trial || t.staged != Tri::No {
                    return Ok(());
                }
                let id = self.truth.next;
                self.truth.next += 1;
                self.truth.b = None;
                let len = self.layout.active * self.page;
                let sizes = [self.page, self.page / 2, self.page / 4, M::WRITE_SIZE * 4];
                let chunk = sizes[chunk as usize % sizes.len()].max(M::WRITE_SIZE);
                self.note(format!(
                    "download {} in chunks of {chunk} bytes (write_firmware)",
                    image_name(id)
                ));
                self.nor(|n| n.quiet = true);
                let mut updater = BlockingFirmwareUpdater::new(
                    FirmwareUpdaterConfig {
                        dfu: self.dfu(),
                        state: self.state(),
                    },
                    &mut word,
                );
                let mut result = Ok(());
                for off in (0..len).step_by(chunk) {
                    let data: Vec<u8> = (off..off + chunk).map(|i| image_byte(id, i)).collect();
                    result = updater.write_firmware(off, &data);
                    if result.is_err() {
                        break;
                    }
                }
                self.nor(|n| n.quiet = false);
                if self.dead() {
                    return Ok(());
                }
                if let Err(e) = result {
                    return Err(self.stuck(format!("write_firmware returned {e:?}")));
                }
                self.truth.b = Some(id);
            }
            Step::MarkUpdated => {
                if self.truth.b.is_none() || self.truth.trial {
                    return Ok(());
                }
                self.note("mark_updated".into());
                let result = BlockingFirmwareState::new(self.state(), &mut word).mark_updated();
                if self.dead() {
                    if self.truth.staged == Tri::No {
                        self.truth.staged = Tri::Maybe;
                    }
                    return Ok(());
                }
                if let Err(e) = result {
                    return Err(self.stuck(format!("mark_updated returned {e:?}")));
                }
                self.truth.staged = Tri::Yes;
            }
            Step::MarkBooted | Step::MarkDfu => {
                let name = if step == Step::MarkBooted {
                    "mark_booted"
                } else {
                    "mark_dfu"
                };
                self.note(name.into());
                let mut fs = BlockingFirmwareState::new(self.state(), &mut word);
                let result = if step == Step::MarkBooted {
                    fs.mark_booted()
                } else {
                    fs.mark_dfu()
                };
                let dead = self.dead();
                let t = &mut self.truth;
                if dead {
                    if t.trial {
                        t.confirm = Tri::Maybe;
                    }
                    if t.staged == Tri::Yes {
                        t.staged = Tri::Maybe;
                    }
                    return Ok(());
                }
                if let Err(e) = result {
                    return Err(self.stuck(format!("{step:?} returned {e:?}")));
                }
                if t.trial {
                    t.a = t.b.take().expect("a trial runs b");
                    t.trial = false;
                    t.confirm = Tri::No;
                }
                t.staged = Tri::No;
            }
        }
        Ok(())
    }

    /// After the last step: no more cuts, three boots must agree on where they end.
    fn settle(&mut self) -> Result<(), Failure> {
        self.nor(|n| n.disarm());
        self.note("settle: no more cuts".into());
        let mut seen = Vec::new();
        for _ in 0..3 {
            self.boot()?;
            seen.push(self.picture());
        }
        if seen[1] != seen[2] {
            return Err(Failure {
                invariant: "I6 boots settle",
                detail: format!(
                    "still changing at the third boot after the last cut:\n  {}\n  {}",
                    seen[1], seen[2]
                ),
                known: None,
            });
        }
        let broken = self.nor(|n| n.broken.clone());
        if !broken.is_empty() {
            return Err(Failure {
                invariant: "I7 the family's write rules hold",
                detail: broken.join("; "),
                known: None,
            });
        }
        Ok(())
    }
}

struct Runner<'a>(&'a [u8]);

impl Visit for Runner<'_> {
    type Out = Outcome;

    fn visit<M: Model>(self) -> Outcome {
        let data = self.0;
        let mut input = Input::new(data);
        input.byte();
        let layout = Layout::decode::<M>(&mut input);
        let mut steps = Vec::new();
        while !input.done() {
            steps.push(Step::decode(&mut input));
        }
        let expect = zoo::expect(M::NAME, M::ERASED);
        let mut sim = Sim::<M>::new(layout);
        sim.note(format!("start: {}", sim.picture()));
        let mut failure = None;
        for &step in &steps {
            if let Err(f) = sim.step(step) {
                failure = Some(f);
                break;
            }
        }
        if failure.is_none() {
            failure = sim.settle().err();
        }
        let ops = sim.nor(|n| n.ops);
        let mut report = String::new();
        let _ = writeln!(report, "model {} ({})", M::NAME, M::SOURCE);
        let _ = writeln!(
            report,
            "  write {} B, erase {} B, erased 0x{:02X}, {:?}{}; expected here: {expect:?}",
            M::WRITE_SIZE,
            M::ERASE_SIZE,
            M::ERASED,
            M::RULE,
            if M::ECC { ", ECC" } else { "" }
        );
        let _ = writeln!(report, "layout {layout:?}");
        let _ = writeln!(report, "steps {steps:?}");
        let _ = writeln!(report, "trace:");
        for line in sim.nor(|n| n.log.clone()) {
            let _ = writeln!(report, "  {line}");
        }
        if let Some(f) = &failure {
            let known = f.known.map(|k| format!(" (known class: {k})")).unwrap_or_default();
            let _ = writeln!(report, "FAILED {}{known}: {}", f.invariant, f.detail);
        }
        let hex: String = data.iter().map(|b| format!("{b:02x}")).collect();
        let _ = writeln!(report, "input {hex}");
        let _ = writeln!(
            report,
            "replay: POWER_FAIL_REPLAY={hex} cargo test --test power_fail replay -- --nocapture"
        );
        Outcome {
            expect,
            failure,
            ops,
            report,
        }
    }
}

/// Runs one input: byte 0 picks the model, the next four the layout, the rest are steps.
pub fn run(data: &[u8]) -> Outcome {
    zoo::with_model(data.first().copied().unwrap_or(0) as usize, Runner(data))
}

struct Name;

impl Visit for Name {
    type Out = (&'static str, u8);

    fn visit<M: Model>(self) -> (&'static str, u8) {
        (M::NAME, M::ERASED)
    }
}

/// Name and erase value of zoo entry `i`.
pub fn model(i: usize) -> (&'static str, u8) {
    zoo::with_model(i, Name)
}
