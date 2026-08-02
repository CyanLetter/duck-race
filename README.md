# Duck Race firmware (Embassy / RP2040)

Async Rust firmware for the original Raspberry Pi Pico. Full design in
[`IMPLEMENTATION.md`](IMPLEMENTATION.md); hardware plan in [`../PLAN.md`](../PLAN.md).

Game: select a duck → GO → four ducks race at randomized speeds → first to trip its
finish switch wins → winner shown → machine re-homes. No payout (prizes by hand).

## Status

Compiles for `thumbv6m-none-eabi` (Embassy 0.9, matched to the `ens160-air-quality`
reference). Core architecture is implemented: motors (DRV8833 shared-reverse), serpentine
LED renderer, input events, the full state machine, flash-persisted calibration, and an
audio stub. **Untested on hardware** — bring-up follows the milestones in
IMPLEMENTATION.md §11. Tune `config.rs` (esp. `COUNTS`) after wiring.

## Toolchain

```bash
rustup target add thumbv6m-none-eabi
cargo install probe-rs-tools    # provides `probe-rs`
```

## Build & flash (via the Raspberry Pi Debug Probe / rs-probe)

`cargo run` builds, flashes over SWD, and streams `defmt` logs over RTT:

```bash
cargo run --release
```

Wire the probe's SWCLK/SWDIO/GND to the Pico's 3-pin debug header. No probe? See the UF2
fallback in `.cargo/config.toml`.

## Layout

| File | Responsibility |
|------|----------------|
| `src/main.rs` | init, `bind_interrupts`, task spawns, runs the game inline |
| `src/config.rs` | pins-as-constants, `COUNTS`/`NUM_LEDS`, duties, timings |
| `src/motors.rs` | DRV8833 abstraction, shared-reverse scheme |
| `src/inputs.rs` | debounced button/switch tasks, `Event`, watchdog-fed `recv` |
| `src/leds.rs` | serpentine `phys_index` mapping, animations, `PioWs2812`, `led_task` |
| `src/game.rs` | state machine (home/attract/select/race/winner), randomization |
| `src/calibrate.rs` | TUNE mode + flash load/save of baselines |
| `src/audio.rs` | `Sound`/`AudioSink` stub → DFPlayer later |

## Pin map

See [`IMPLEMENTATION.md` §3](IMPLEMENTATION.md). Summary: motors GP2–GP7, LEDs GP9,
select GP10–13, GO GP14, TUNE GP15/16, start switches GP17–20, finish GP21/22/26/27,
audio (future) GP0/1.

## Controls

- **Duck-select ×4 + GO** — pick a duck, start the race.
- **TUNE mode** — hold **GO at power-up**. Select a lane, **UP/DOWN** adjust its baseline
  duty, **GO** runs a test race on it, **hold GO** saves to flash and exits.
