//! A NOR flash array with the write rules of one family, power cuts at any program or erase, and a log of
//! every operation.

use std::fmt::Write as _;
use std::marker::PhantomData;

use embedded_storage::nor_flash::{ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash};

/// How a program unit may be written once it is not erased.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rule {
    /// The unit must be erased, except that all zeros can always be programmed. Anything else is refused.
    OnceOrZero,
    /// Bits only go from 1 to 0, any number of times; a 1 over a 0 silently stays 0.
    BitsToZero,
    /// Erases to 0x00. The unit must be zero; a write to a unit that is not zero is refused.
    NotZeroRefused,
    /// Erases to 0x00. A write over programmed data ORs into it.
    Or,
}

/// A flash family.
pub trait Model: 'static {
    /// Name in reports and for `POWER_FAIL_MODEL`.
    const NAME: &'static str;
    /// Where the rules come from.
    const SOURCE: &'static str;
    /// `NorFlash::WRITE_SIZE` of the embassy driver.
    const WRITE_SIZE: usize;
    /// `NorFlash::ERASE_SIZE` of the embassy driver.
    const ERASE_SIZE: usize;
    /// The unit the write rule and ECC apply to.
    const UNIT: usize;
    /// What an erase leaves.
    const ERASED: u8;
    /// The write rule.
    const RULE: Rule;
    /// A unit left half-programmed or half-erased by a cut faults on read.
    const ECC: bool;
}

/// Error of a [`Nor`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SimError {
    /// Power is off.
    PowerCut,
    /// Uncorrectable ECC error on read.
    Ecc,
    /// The family's controller refused the program (PGERR, PROGERR, NOTZEROERR).
    Refused,
    /// Out of bounds or misaligned.
    Bounds,
}

impl NorFlashError for SimError {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            SimError::Bounds => NorFlashErrorKind::OutOfBounds,
            _ => NorFlashErrorKind::Other,
        }
    }
}

/// splitmix64.
#[derive(Clone)]
pub struct Rng(pub u64);

impl Rng {
    pub fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    pub fn below(&mut self, n: usize) -> usize {
        (self.next() % n.max(1) as u64) as usize
    }
}

/// Named address ranges, for the log.
#[derive(Clone, Default)]
pub struct Regions(pub Vec<(&'static str, usize, usize)>);

impl Regions {
    pub fn name(&self, at: usize) -> String {
        for &(name, start, len) in &self.0 {
            if (start..start + len).contains(&at) {
                return format!("{name}+{:#x}", at - start);
            }
        }
        format!("{at:#x}")
    }

    pub fn contains(&self, name: &str, at: usize) -> bool {
        self.0
            .iter()
            .any(|&(n, start, len)| n == name && (start..start + len).contains(&at))
    }
}

/// The flash array.
pub struct Nor<M> {
    pub mem: Vec<u8>,
    /// Per unit: faults on read.
    pub bad: Vec<bool>,
    /// Programs and erases so far (each sector of an erase counts once).
    pub ops: u64,
    cut: Option<(u64, Rng)>,
    pub dead: bool,
    /// The last error returned, with its address.
    pub last_error: Option<(SimError, usize)>,
    /// The last program a cut tore: where it started and what it was writing.
    pub torn: Option<(usize, Vec<u8>)>,
    /// A family rule was broken (silently, as the chip would): what and where.
    pub broken: Vec<String>,
    pub log: Vec<String>,
    /// Leave plain programs and erases out of the log (cuts and refusals are always logged).
    pub quiet: bool,
    pub regions: Regions,
    ops_since_power_on: u64,
    model: PhantomData<M>,
}

/// More operations than this without a power cut is a loop.
const OP_BUDGET: u64 = 100_000;

impl<M: Model> Nor<M> {
    pub fn new(size: usize, regions: Regions) -> Self {
        Self {
            mem: vec![M::ERASED; size],
            bad: vec![false; size / M::UNIT],
            ops: 0,
            cut: None,
            dead: false,
            last_error: None,
            torn: None,
            broken: Vec::new(),
            log: Vec::new(),
            quiet: false,
            regions,
            ops_since_power_on: 0,
            model: PhantomData,
        }
    }

    /// Cuts power during operation number `at` (counted from the start), torn as `tear` says.
    pub fn arm(&mut self, at: u64, tear: u64) {
        self.cut = Some((at, Rng(tear)));
    }

    pub fn disarm(&mut self) {
        self.cut = None;
    }

    pub fn power_on(&mut self) {
        self.dead = false;
        self.ops_since_power_on = 0;
    }

    pub fn note(&mut self, line: String) {
        self.log.push(line);
    }

    fn fail(&mut self, error: SimError, at: usize) -> SimError {
        self.last_error = Some((error, at));
        error
    }

    /// Starts an operation: `Some(rng)` when the cut hits it.
    fn start(&mut self) -> Result<Option<Rng>, SimError> {
        if self.dead {
            return Err(SimError::PowerCut);
        }
        let n = self.ops;
        self.ops += 1;
        self.ops_since_power_on += 1;
        assert!(
            self.ops_since_power_on < OP_BUDGET,
            "{}: more than {OP_BUDGET} flash operations without a power cut: a loop",
            M::NAME
        );
        match self.cut.take() {
            Some((at, rng)) if at == n => Ok(Some(rng)),
            other => {
                self.cut = other;
                Ok(None)
            }
        }
    }

    /// Programming moves bits this way: toward 0 on 0xFF-erased flash, toward 1 on 0x00-erased.
    fn partial(old: u8, new: u8, r: u8) -> u8 {
        if M::ERASED == 0xFF {
            old & (new | r)
        } else {
            old | (new & r)
        }
    }

    fn program_unit(&mut self, at: usize, new: &[u8]) -> Result<(), SimError> {
        let unit = at / M::UNIT;
        let cur = &self.mem[at..at + M::UNIT];
        let erased = cur.iter().all(|&b| b == M::ERASED) && !self.bad[unit];
        match M::RULE {
            Rule::OnceOrZero => {
                if !erased && new.iter().any(|&b| b != 0) {
                    return Err(SimError::Refused);
                }
                self.mem[at..at + M::UNIT].copy_from_slice(new);
                self.bad[unit] = false;
            }
            Rule::BitsToZero => {
                for (i, &b) in new.iter().enumerate() {
                    let cell = &mut self.mem[at + i];
                    if *cell & b != b {
                        let line = format!("{}: program would need a bit 0 -> 1", self.regions.name(at + i));
                        self.broken.push(line);
                    }
                    *cell &= b;
                }
            }
            Rule::NotZeroRefused => {
                if cur.iter().any(|&b| b != 0) {
                    return Err(SimError::Refused);
                }
                self.mem[at..at + M::UNIT].copy_from_slice(new);
            }
            Rule::Or => {
                if !erased && new.iter().any(|&b| b != 0) {
                    let line = format!(
                        "{}: write over programmed data (undocumented; modelled as OR)",
                        self.regions.name(at)
                    );
                    self.broken.push(line);
                }
                for (i, &b) in new.iter().enumerate() {
                    self.mem[at + i] |= b;
                }
            }
        }
        Ok(())
    }

    fn tear_program(&mut self, at: usize, new: &[u8], rng: &mut Rng) -> &'static str {
        match rng.below(3) {
            0 => "not started",
            v => {
                for (i, &b) in new.iter().enumerate() {
                    let r = rng.next() as u8;
                    self.mem[at + i] = Self::partial(self.mem[at + i], b, r);
                }
                if M::ECC && v == 2 {
                    self.bad[at / M::UNIT] = true;
                    "half programmed, ECC fault"
                } else {
                    "half programmed"
                }
            }
        }
    }

    fn tear_erase(&mut self, block: usize, rng: &mut Rng) {
        for at in (block..block + M::ERASE_SIZE).step_by(M::UNIT) {
            match rng.below(4) {
                0 => {}
                1 => {
                    self.mem[at..at + M::UNIT].fill(M::ERASED);
                    self.bad[at / M::UNIT] = false;
                }
                v => {
                    for cell in &mut self.mem[at..at + M::UNIT] {
                        let r = rng.next() as u8;
                        *cell = if M::ERASED == 0xFF { *cell | r } else { *cell & r };
                    }
                    // ECC catches most half-erased units; one that is nearly erased can read wrong unflagged.
                    if M::ECC && v == 2 {
                        self.bad[at / M::UNIT] = true;
                    }
                }
            }
        }
    }

    /// The first byte of every `M::WRITE_SIZE` word, `!!` for a unit that faults.
    pub fn words(&self, start: usize, words: usize) -> String {
        let mut out = String::new();
        for w in 0..words {
            let at = start + w * M::WRITE_SIZE;
            if self.bad[at / M::UNIT] {
                out.push_str("!! ");
            } else {
                let _ = write!(out, "{:02X} ", self.mem[at]);
            }
        }
        out.trim_end().to_string()
    }
}

impl<M: Model> ErrorType for Nor<M> {
    type Error = SimError;
}

impl<M: Model> ReadNorFlash for Nor<M> {
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), SimError> {
        let at = offset as usize;
        if self.dead {
            return Err(SimError::PowerCut);
        }
        if at + bytes.len() > self.mem.len() {
            return Err(self.fail(SimError::Bounds, at));
        }
        if bytes.is_empty() {
            return Ok(());
        }
        if let Some(unit) = (at / M::UNIT..(at + bytes.len()).div_ceil(M::UNIT)).find(|&u| self.bad[u]) {
            return Err(self.fail(SimError::Ecc, unit * M::UNIT));
        }
        bytes.copy_from_slice(&self.mem[at..at + bytes.len()]);
        Ok(())
    }

    fn capacity(&self) -> usize {
        self.mem.len()
    }
}

impl<M: Model> NorFlash for Nor<M> {
    const WRITE_SIZE: usize = M::WRITE_SIZE;
    const ERASE_SIZE: usize = M::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), SimError> {
        let (from, to) = (from as usize, to as usize);
        if from > to || to > self.mem.len() || !from.is_multiple_of(M::ERASE_SIZE) || !to.is_multiple_of(M::ERASE_SIZE)
        {
            return Err(self.fail(SimError::Bounds, from));
        }
        for block in (from..to).step_by(M::ERASE_SIZE) {
            let n = self.ops;
            let cut = self.start()?;
            let name = self.regions.name(block);
            match cut {
                None => {
                    self.mem[block..block + M::ERASE_SIZE].fill(M::ERASED);
                    self.bad[block / M::UNIT..(block + M::ERASE_SIZE) / M::UNIT].fill(false);
                    if !self.quiet {
                        self.log.push(format!("op {n:3}: erase   {name}"));
                    }
                }
                Some(mut rng) => {
                    self.tear_erase(block, &mut rng);
                    self.dead = true;
                    self.log
                        .push(format!("op {n:3}: erase   {name}  <-- POWER CUT, erase torn"));
                    return Err(self.fail(SimError::PowerCut, block));
                }
            }
        }
        Ok(())
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), SimError> {
        let at = offset as usize;
        if at + bytes.len() > self.mem.len()
            || !at.is_multiple_of(M::WRITE_SIZE)
            || !bytes.len().is_multiple_of(M::WRITE_SIZE)
        {
            return Err(self.fail(SimError::Bounds, at));
        }
        let n = self.ops;
        let cut = self.start()?;
        let name = self.regions.name(at);
        let units = bytes.len() / M::UNIT;
        let mut cut = cut.map(|mut rng| (rng.below(units), rng));
        for (i, new) in bytes.chunks(M::UNIT).enumerate() {
            let unit_at = at + i * M::UNIT;
            if let Some((_, rng)) = cut.as_mut().filter(|(torn, _)| *torn == i) {
                let what = self.tear_program(unit_at, new, rng);
                self.torn = Some((at, bytes.to_vec()));
                self.dead = true;
                self.log.push(format!(
                    "op {n:3}: program {name} {} bytes {:02X?}  <-- POWER CUT at unit {i}: {what}",
                    bytes.len(),
                    &bytes[..bytes.len().min(8)]
                ));
                return Err(self.fail(SimError::PowerCut, unit_at));
            }
            if let Err(e) = self.program_unit(unit_at, new) {
                let old = self.mem[unit_at..unit_at + M::UNIT].to_vec();
                self.log.push(format!(
                    "op {n:3}: program {name} {} bytes {:02X?}  <-- REFUSED by the controller at {} (holds {:02X?})",
                    bytes.len(),
                    &bytes[..bytes.len().min(8)],
                    self.regions.name(unit_at),
                    old
                ));
                return Err(self.fail(e, unit_at));
            }
        }
        if self.quiet {
        } else if bytes.len() <= 2 * M::UNIT.max(8) {
            self.log.push(format!("op {n:3}: program {name} {:02X?}", bytes));
        } else {
            self.log.push(format!("op {n:3}: program {name} {} bytes", bytes.len()));
        }
        Ok(())
    }
}
