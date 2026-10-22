//! The flash families, one entry per set of rules. Sources are the reference manuals and datasheets.

use super::flash::{Model, Rule};

macro_rules! model {
    ($(#[$doc:meta])* $name:ident, $source:literal, write $write:expr, erase $erase:expr, unit $unit:expr,
     erased $erased:expr, $rule:expr, ecc $ecc:expr) => {
        $(#[$doc])*
        pub struct $name;
        impl Model for $name {
            const NAME: &'static str = stringify!($name);
            const SOURCE: &'static str = $source;
            const WRITE_SIZE: usize = $write;
            const ERASE_SIZE: usize = $erase;
            const UNIT: usize = $unit;
            const ERASED: u8 = $erased;
            const RULE: Rule = $rule;
            const ECC: bool = $ecc;
        }
    };
}

model!(
    /// STM32F0/F1/F3: half-words; a programmed one is skipped with PGERR, except 0x0000. 1 KB pages.
    F1, "RM0008 / PM0075 Rev 2 §2.3.3, RM0091 Rev 10 §3.2.2",
    write 2, erase 1024, unit 2, erased 0xFF, Rule::OnceOrZero, ecc false
);
model!(
    /// STM32G0 (and G4, L4, WB, WL): double words with ECC, PROGERR unless erased or all zero; ECCD on a torn
    /// unit (an NMI on the chip, a read error with embassy-stm32's ECC handling). 2 KB pages.
    G0, "RM0444 Rev 6 §3.3.3, §3.3.6, §3.3.8",
    write 8, erase 2048, unit 8, erased 0xFF, Rule::OnceOrZero, ecc true
);
model!(
    /// RP2040/RP2350 QSPI NOR (W25Q16JV, W25Q32RV): bytes, bits only to 0, 4 KB sectors.
    Qspi, "RP2040/RP2350 datasheets (boot ROM flash functions), W25Q16JV Rev J Page Program",
    write 1, erase 4096, unit 1, erased 0xFF, Rule::BitsToZero, ecc false
);
model!(
    /// STM32L0: erases to 0x00, a write to a word that is not zero is not performed (NOTZEROERR). 128 B pages.
    L0, "RM0377 Rev 10 §3.3 (erase sets bits to 0; NOTZEROERR)",
    write 4, erase 128, unit 4, erased 0x00, Rule::NotZeroRefused, ecc false
);
model!(
    /// STM32L1: erases to 0x00 (inferred), a write over programmed data is not documented: modelled as OR and
    /// reported. 256 B pages.
    L1, "RM0038 Rev 18 §3 (no not-zero flag; overwrite undocumented)",
    write 4, erase 256, unit 4, erased 0x00, Rule::Or, ecc false
);

/// What the test expects of an entry in this build.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Expect {
    /// Every scenario must pass (known classes aside).
    Pass,
    /// A known finding: the test passes only if the sweep reproduces it.
    Finding(&'static str),
    /// Not a configuration anyone can build (erase value of the chip vs `flash-erase-zero`).
    Skip,
}

const ERASE_ZERO: bool = cfg!(feature = "flash-erase-zero");

/// The expectation for a model with erase value `erased` in this build.
pub const fn expect(name: &str, erased: u8) -> Expect {
    let zero_chip = erased == 0x00;
    match (zero_chip, ERASE_ZERO) {
        (false, false) => Expect::Pass,
        (false, true) => Expect::Skip,
        (true, false) => Expect::Finding(
            "flash erases to 0x00 but embassy-boot is built without `flash-erase-zero`, as the L0/L1 boot examples are",
        ),
        (true, true) => {
            if name.len() == 2 && name.as_bytes()[1] == b'0' {
                Expect::Finding(
                    "with `flash-erase-zero`, the magic is cleared with 0xFF over 0xF0, which the L0 refuses",
                )
            } else {
                Expect::Finding("with `flash-erase-zero`, the magic is cleared with 0xFF over 0xF0, undocumented on L1")
            }
        }
    }
}

/// Something to run once per model type.
pub trait Visit {
    type Out;
    fn visit<M: Model>(self) -> Self::Out;
}

/// The zoo: entry `i` (taken modulo the size, so any byte picks one).
pub fn with_model<V: Visit>(i: usize, v: V) -> V::Out {
    match i % COUNT {
        0 => v.visit::<F1>(),
        1 => v.visit::<G0>(),
        2 => v.visit::<Qspi>(),
        3 => v.visit::<L0>(),
        _ => v.visit::<L1>(),
    }
}

/// Number of entries in [`with_model`].
pub const COUNT: usize = 5;
