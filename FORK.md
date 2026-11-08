# romixlab/embassy

Our fork of [embassy-rs/embassy](https://github.com/embassy-rs/embassy): upstream plus the small fixes our firmware needs before (or instead of) their upstream release.

## Branches

- `main` is what our firmware depends on: an upstream commit with every topic branch merged in. It only moves forward (merges, never a rebase or a force push), so every rev a `Cargo.lock` ever pinned stays reachable.
- `t/<slug>` holds one fix, starting from an upstream commit, so it can go upstream as a PR on its own. A fix that needs another one starts from that topic branch and says so below.
- Older branches (`donguru-ring-copy`, `upstream/h7-fracn`, ...) are kept for the revs old lockfiles pin. Nothing new goes there.

## Topics in main

| Branch | What | Upstream |
|---|---|---|
| `t/dma-ring-copy` | stm32/dma: the ring buffer copies out in up to two runs, no division per element (the header UART of Donguru at 5.8 Mbaud on the M0+) | not offered |
| `t/h7-fracn` | stm32/rcc h7: `Pll::fracn` on every STM32H7, VCO in integers (B135) | not offered |
| `t/tim-psc-round` | stm32/time-driver: TIM prescaler rounded to the nearest tick (a fractional PLL ran embassy-time 0.39 % fast) | not offered |

## Working on it

- **A new fix:** branch `t/<slug>` from the upstream commit `main` is on, commit, merge it into `main` with `--no-ff`, add a row above.
- **A newer upstream:** merge upstream `main` (or a release tag) into `main`, never rebase. When upstream merged a topic, its row goes, and the branch stays.
- **Changing a topic:** a new commit on its branch, merged into `main` again.
- **Downstream:** `[patch.crates-io]` with a plain `git = "https://github.com/romixlab/embassy"` (no branch, tag or rev: `main` is the default branch) for every embassy crate the firmware uses. `Cargo.lock` holds the rev, and `cargo update -p embassy-stm32` (or `-p embassy-rp`) moves it.
