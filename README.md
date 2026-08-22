# Duck Race firmware (Embassy / RP2040)

Async Rust firmware for the original Raspberry Pi Pico. Full design in
[`IMPLEMENTATION.md`](IMPLEMENTATION.md); hardware plan in [`../PLAN.md`](../PLAN.md).

Game (side-on cabinet, lanes stacked): select a duck → GO → four ducks race at speeds that
vary mid-race → first to trip its finish switch wins → win/lose sting → machine re-homes.
No payout (prizes by hand).

## Status

All five build configurations compile clean for `thumbv6m-none-eabi` (Embassy 0.9, matched
to the `ens160-air-quality` reference). On hardware: all four lanes assembled with bumpers
and limit switches, N20 motors driving, LED rows verified, DY-SV8F audio playing. Duty
constants in `config.rs` are tuned against the real rig — re-check `FLOOR_PCT` after any
belt-tension work.

## Toolchain

```bash
rustup target add thumbv6m-none-eabi
```

```bash
cargo install probe-rs-tools
```

## Build & flash (via the Raspberry Pi Debug Probe / rs-probe)

`cargo run` builds, flashes over SWD, and streams `defmt` logs over RTT:

```bash
cargo run --release
```

Wire the probe's SWCLK/SWDIO/GND to the Pico's 3-pin debug header. No probe? See the UF2
fallback in `.cargo/config.toml`.

## Bring-up modes

Each is a cargo feature; **enable exactly one** (a compile-time guard enforces it). The
default build with no features is the full game. Details in IMPLEMENTATION.md §11.

| Feature | What it does | Needs |
|---------|--------------|-------|
| `test-motors` | Hold-to-run jog. Duck button picks the lane, TUNE up/down = forward/reverse. | one motor |
| `test-lane` | Drive one lane to its finish switch, then home. GO starts it. | lane + both switches |
| `test-leds` | Serpentine walk: dot through the chain, rows in duck colours, per-row start→finish. | LED rows |
| `test-audio` | Clip check. Duck buttons play bet stings, GO cycles all seven, TUNE up/down = volume. | audio module + speaker |

```bash
cargo run --release --features test-leds
```

## Layout

| File | Responsibility |
|------|----------------|
| `src/main.rs` | init, `bind_interrupts`, task spawns, runs the game inline |
| `src/config.rs` | every tuning constant: duties, timings, LED counts, chase feel |
| `src/motors.rs` | DRV8833 abstraction, shared-reverse scheme |
| `src/inputs.rs` | debounced button/switch tasks, `Event`, switch level cache, watchdog-fed `recv` |
| `src/leds.rs` | serpentine `phys_index` mapping, animations, `PioWs2812`, `led_task` |
| `src/game.rs` | state machine (home/attract/select/race/winner), race speed segments |
| `src/calibrate.rs` | TUNE mode + flash load/save of baselines |
| `src/audio.rs` | `Sound`/`AudioSink`, DY-SV8F UART backend |
| `src/bringup.rs` | the feature-gated test modes |

## Pin map

Banked in groups of four so each cluster is one solderable header — see
[`IMPLEMENTATION.md` §3](IMPLEMENTATION.md) for the full table and the reasoning.

| Pins | Function |
|------|----------|
| GP0 / GP1 | audio UART0 TX / RX → DY-SV8F |
| GP2–GP5 | motor forward PWM, lanes 0–3 |
| GP6–GP9 | **finish** limit switches 0–3 |
| GP10–GP13 | duck-select buttons 0–3 |
| GP14 | GO (hold at boot → TUNE) |
| GP15 | spare |
| GP16 / GP17 | TUNE up / down |
| GP18–GP21 | **home** limit switches 0–3 |
| GP22 | WS2812 data → 74AHCT125 |
| GP26 / GP27 / GP28 | shared reverse PWM / nSLEEP / nFAULT |

## Controls

- **Duck-select ×4 + GO** — pick a duck, start the race.
- **TUNE mode** — hold **GO at power-up**. Select a lane, **UP/DOWN** adjust its baseline
  duty, **GO** runs a test race on it, **hold GO** saves to flash and exits.

## Audio clips — deploying to the DY-SV8F

Masters live in [`../Audio/04_NamedForExport/`](../Audio/04_NamedForExport). The module
selects a track by its 5-digit filename:

| File | Sound |
|------|-------|
| `00001.mp3`–`00004.mp3` | bet sting, one per duck (lane 0–3) |
| `00005.mp3` | race bed |
| `00006.mp3` | finish — player's duck **won** |
| `00007.mp3` | finish — player's duck **lost** |

### ⚠️ macOS writes hidden files that break track selection

Copying to the module's FAT volume in Finder creates an AppleDouble sidecar (`._00001.mp3`)
next to every clip. **This module enumerates files in directory order, so the sidecars
count as tracks** — the file list becomes twice as long and every requested track number
lands on the wrong file:

```
00001.mp3    → index 1  ✅ plays
._00001.mp3  → index 2  ❌ silence
00002.mp3    → index 3  ✅ plays   ← you asked for track 2
```

The symptom is unmistakable: **odd track numbers play the wrong clip, even ones are
silent.** `.DS_Store`, `.Trashes`, `.Spotlight-V100` and `.fseventsd` cause the same thing.

**Check** the mounted module:

```bash
ls -la /Volumes/YOUR_MODULE_VOLUME
```

**Clean** it:

```bash
find /Volumes/YOUR_MODULE_VOLUME \( -name '._*' -o -name '.DS_Store' \) -delete
```

```bash
rm -rf /Volumes/YOUR_MODULE_VOLUME/.Trashes /Volumes/YOUR_MODULE_VOLUME/.Spotlight-V100 /Volumes/YOUR_MODULE_VOLUME/.fseventsd
```

**Avoid it** by copying with `cp -X` (excludes extended attributes) instead of dragging in
Finder, one file at a time in numeric order so filename order and write order agree:

```bash
for f in ../Audio/04_NamedForExport/0000*.mp3; do cp -X "$f" /Volumes/YOUR_MODULE_VOLUME/; done
```

Then `ls -la` again to confirm only the seven `.mp3` files are present before ejecting, and
verify with `cargo run --release --features test-audio` — GO steps through all seven in
order and logs each track number, so a mismatch is obvious.
