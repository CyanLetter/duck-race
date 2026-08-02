# Duck Race — Firmware Implementation Plan

Target: **Raspberry Pi Pico (RP2040, plain — not W)**, Embassy async Rust, flashed/debugged
over a **Raspberry Pi Debug Probe (rs-probe)** with `probe-rs` + `defmt` RTT.

Game (top-down, pinball-style cabinet): player presses a duck-select button, presses GO,
four ducks race down the V-slot lanes at randomized speeds, first to trip its finish
switch wins, the winner is shown on the lane LEDs, then the machine re-homes. **No payout
logic, no odds, no bets** — prizes are handed out by hand at the convention. This keeps
the state machine simple: `select → race → show winner → reset`.

> This document is the plan. The existing `firmware/` scaffold (`Cargo.toml`, `main.rs`,
> `ws2812.rs`) predates it and the reference-project findings below — it will be
> **regenerated** to match this plan (Embassy 0.9, DRV8833, built-in `PioWs2812`) when we
> start building.

---

## 1. Toolchain & dependencies — matched to your reference project

Aligned to `PiPico/Rust/ens160-air-quality` so the tooling is identical to what you run
today. Differences from that project (it's a Pico **W**) are called out in §2.

- **Edition 2024**, **Rust 1.92 stable**, target `thumbv6m-none-eabi`.
- `.cargo/config.toml`: `runner = "probe-rs run --chip RP2040"`; linker args
  (`--nmagic`, `-Tlink.x`, `-Tlink-rp.x`, `-Tdefmt.x`) emitted from `build.rs` via
  `cargo:rustc-link-arg-bins` (same as your reference).
- **Crates (crates.io, same generation as the reference):**
  - `embassy-executor 0.9`, `embassy-rp 0.9` (feature `rp2040`), `embassy-time 0.5`,
    `embassy-sync 0.7`, `embassy-futures 0.1`
  - `smart-leds 0.4`, `libm 0.2` (animation math)
  - `defmt 1.0`, `defmt-rtt 1.0`, `panic-probe 1.0`
  - `cortex-m 0.7`, `cortex-m-rt 0.7`, `critical-section 1.1`, `static_cell 2.1`,
    `portable-atomic 1.5`
  - **No** `alloc`/`embedded-alloc`, **no** `cyw43*`, **no** `embassy-net`/`reqwless`
    (those were for WiFi/TLS on the W — we don't need a heap).
  - For flash-persisted tuning (§6): `sequential-storage` **or** raw `embassy_rp::flash`.
- **WS2812 driver is built in** — `embassy_rp::pio_programs::ws2812::{PioWs2812,
  PioWs2812Program, Grb}`. No vendored PIO driver. `PioWs2812::new(&mut common, sm, dma,
  pin, &program)` then `ws.write(&[RGB8; N]).await`. `N` is a **const generic** (see §5).

## 2. Deltas from the Pico-W reference

- **PIO0 is free.** The reference used PIO1 for LEDs because CYW43 (WiFi) owned PIO0. We
  have no WiFi, so **use PIO0 SM0** for WS2812.
- **GP23/24/25 differ.** On the W those drove the CYW43 (`PIN_23` pwr, `PIN_25` cs). On the
  plain Pico: GP23 = SMPS power-save, GP24 = VBUS sense, **GP25 = onboard LED** (usable as
  a status/heartbeat LED). GP29 = VSYS/ADC. Avoid 23/24 for I/O; GP25 is our status LED.
- **No heap, no watchdog-for-network.** Keep the `Watchdog` (good hang protection — feed
  it in the game loop) but drop the TLS heap init.
- Reuse the reference's patterns verbatim: `bind_interrupts!`, `Pio::new(p.PIO0, Irqs)`
  destructure, `Adc::new(p.ADC, Irqs, ...)`, `static_cell::StaticCell`, RTT logging.

### 2.1 Critical review of the reference — the LED-stall bug, and how we avoid it

The reference has a real defect worth understanding before we copy its shape.

**Root cause (not what its CLAUDE.md claims):** Embassy's default executor is
**single-core and cooperative — it never preempts.** The reference moved networking into
`api_submit_task` believing that would stop it blocking LED animation. It doesn't: the TLS
handshake / HTTP is **CPU-bound crypto that rarely `.await`s**, so while it runs it
monopolizes the core and *every* other task — including LED animation — is starved.
Task separation buys concurrency only across `.await` points; it gives **no preemption**.
Secondarily, the reference drives `led_controller.update()` inline in one large main loop
that also runs a multi-`await` sensor-read block, so frames are dropped during ordinary
I2C reads too.

**The "unusual" LED control:** (1) animation is inline in that mega-loop rather than a
dedicated ticker task; (2) the Pico-W **onboard status LED is toggled through the CYW43
WiFi chip** (`control.gpio_set(0, …)` over PIO0 SPI — the W has no direct GPIO to it), so
that LED contends with WiFi; (3) `update()` computes `powf` gamma + `sinf` **per channel,
per LED, every frame** — costly soft-float on the FPU-less M0+ (fine for 16 LEDs, wasteful
for our ~90).

**How this plan avoids all of it:**
- **Dedicated `led_task` on a `Ticker`**, fed only a small `RaceView` snapshot — decoupled
  from game logic (§7).
- **The important part:** a separate task is *not by itself* a fix on a cooperative
  single-core executor. We're safe because **the duck-race has no long CPU-bound section
  during play** — motor PWM, RNG, and switch handling are all trivial and yield. So nothing
  can starve the renderer mid-race. (Contrast: the reference had seconds-long crypto.)
- **The one CPU-bound op we do have is flash erase/write** for calibration save, which
  pauses XIP and stalls everything for tens of ms. We confine it to **TUNE mode only, never
  during a race/animation** (§6.1) — so the hitch is invisible.
- **Precompute a 256-entry gamma LUT** (`u8→u8`) instead of `powf` per channel per frame;
  keep `sinf` out of the inner loop (LUT / fixed-point). Removes soft-float from the hot
  path for ~90 LEDs.
- **Documented escape hatch (not needed now):** if a future feature ever runs CPU-bound
  work concurrently with animation, move the renderer to a **high-priority interrupt
  executor** (`embassy/examples/rp/src/bin/interrupt.rs`) or the **second core**
  (`.../multicore.rs`) — the preemption the reference lacked. We don't need it for this game.

> Grounding refs in the embassy repo: `examples/rp/src/bin/pio_ws2812.rs` (LED driver
> usage), `pwm.rs` (Config/`compare_a`/`set_config`/`SetDutyCycle`), `interrupt.rs` &
> `multicore.rs` (preemptible/isolated rendering), and `embassy-rp/src/pio_programs/ws2812.rs`
> (the built-in `PioWs2812<P, S, N, ORDER>` — `N` is a **const generic**, `write` takes
> `&[RGB8; N]` and stack-builds a `[u32; N]`, so total LED count is fixed at compile time).

---

## 3. Pin map (RP2040)

Reserved by the board: GP23/24 (SMPS/VBUS), GP25 (onboard LED), GP29 (VSYS). SWD debug
(SWCLK/SWDIO/GND) uses the dedicated 3-pin debug header, **not** GPIO.

| GPIO | Function | Peripheral / notes |
|------|----------|--------------------|
| GP0  | Audio UART0 **TX** → DFPlayer RX | *reserved, future (§9)* |
| GP1  | Audio UART0 **RX** ← DFPlayer TX | *reserved, future (optional)* |
| GP2  | Motor 0 IN1 (forward PWM) | PWM slice1 A |
| GP3  | Motor 1 IN1 (forward PWM) | PWM slice1 B |
| GP4  | Motor 2 IN1 (forward PWM) | PWM slice2 A |
| GP5  | Motor 3 IN1 (forward PWM) | PWM slice2 B |
| GP6  | **Shared** IN2 (reverse/home PWM) → all 4 motors | PWM slice3 A |
| GP7  | DRV8833 nSLEEP (enable, both boards) | GPIO out |
| GP8  | DRV8833 nFAULT (both, wired-OR, pull-up) | GPIO in — *optional; else spare* |
| GP9  | WS2812 data → 74AHCT125 → LED chain | PIO0 SM0 |
| GP10 | Duck-select button 0 | Input, `Pull::Up` |
| GP11 | Duck-select button 1 | Input, `Pull::Up` |
| GP12 | Duck-select button 2 | Input, `Pull::Up` |
| GP13 | Duck-select button 3 | Input, `Pull::Up` |
| GP14 | GO button (also: hold at boot → TUNE mode) | Input, `Pull::Up` |
| GP15 | TUNE **UP / +** | Input, `Pull::Up` |
| GP16 | TUNE **DOWN / −** | Input, `Pull::Up` |
| GP17 | Start (home) limit switch — lane 0 | Input, `Pull::Up` |
| GP18 | Start limit — lane 1 | Input, `Pull::Up` |
| GP19 | Start limit — lane 2 | Input, `Pull::Up` |
| GP20 | Start limit — lane 3 | Input, `Pull::Up` |
| GP21 | End (finish) limit — lane 0 | Input, `Pull::Up` |
| GP22 | End limit — lane 1 | Input, `Pull::Up` |
| GP26 | End limit — lane 2 | Input (ADC0-capable, used digital) |
| GP27 | End limit — lane 3 | Input (ADC1-capable, used digital) |
| GP28 | **Spare** (ADC2) | future master-volume pot, or a global speed trim |
| GP25 | Onboard LED — heartbeat/status | Output |

24 pins hard-assigned; GP8 (nFAULT) and GP28 are flexible spares. Everything fits on a
bare Pico with no I/O expander.

> **Pin-pressure alternative:** if you later want fully independent per-motor control
> (individual brake/home-stop, see §4) that needs 8 motor pins instead of 5, move the 8
> limit switches onto an **MCP23017 I²C expander** (2 pins, frees 6). Not needed for the
> plan as written.

---

## 4. Motor control — DRV8833 (×2)

Each **DRV8833** is a dual H-bridge → 2 motors per board, so **two boards** for four lanes.
It is **not a bus device** — no I²C/SPI. You drive it directly with logic-level PWM +
direction pins. Per H-bridge there are two inputs (INx1/INx2) and two outputs (OUTx1/OUTx2).

### 4.1 "In-in" drive & truth table (per motor)

We use in-in (PWM) mode, fast-decay:

| IN1 | IN2 | Result |
|-----|-----|--------|
| PWM | 0   | **Forward** at duty |
| 0   | PWM | **Reverse** at duty (homing) |
| 0   | 0   | Coast (off) |
| 1   | 1   | Brake (fast stop) |

### 4.2 Shared-IN2 wiring (5 pins for 4 motors)

All lanes always race **forward together** and home **reverse together**, so the reverse
input can be shared. Give each motor its own forward-PWM pin (independent speed) and tie
all four IN2 pins to one shared reverse-PWM pin:

| DRV8833 | Motor | IN1 ← | IN2 ← | OUT → |
|---------|-------|-------|-------|-------|
| Board A | 0 | GP2 | GP6 (shared) | AOUT1/2 → lane-0 motor |
| Board A | 1 | GP3 | GP6 (shared) | BOUT1/2 → lane-1 motor |
| Board B | 2 | GP4 | GP6 (shared) | AOUT1/2 → lane-2 motor |
| Board B | 3 | GP5 | GP6 (shared) | BOUT1/2 → lane-3 motor |

Both boards: **nSLEEP → GP7** (high = enabled), **VM → 5 V motor rail**, **GND → common
ground**, **nFAULT → GP8** (open-drain, one pull-up, wired-OR).

- **Race:** IN2 (GP6) = 0; each IN1 = PWM(speed_i) → all forward, independent speeds.
- **Home/reset:** all IN1 = 0; IN2 (GP6) = PWM(homing) → all reverse together.
- **Stop at finish:** all IN1 = 1 **and** GP6 = 1 → brake all (race is over for everyone,
  so braking the shared line is exactly right — stops the slam into the end bumper).

**Constraint this imposes:** you can't drive two lanes in *opposite* directions at once.
That never happens in this game, but it's why homing is "all-together" (§7).

### 4.3 PWM setup

- GP2/GP3 = slice1 A/B, GP4/GP5 = slice2 A/B, GP6 = slice3 A. A/B channels on a slice
  share frequency (good — we want one motor PWM frequency) with **independent compare**
  (independent duty). Set `Config.top` for **~20 kHz** (above audible) so the motors don't
  whine; duty = `top × pct/100`.
- **PWM floor ~35%**: DC motors won't creep smoothly near 0. Never command a moving motor
  below the floor.
- **Launch kick**: ~90% for ~100–150 ms at race start to break static friction, then drop
  to the lane's target duty.

### 4.4 Motor abstraction (`motors.rs`)

```
struct Motors { /* 3 Pwm slice handles, nSLEEP Output, shared config */ }
impl Motors {
    fn enable(&mut self, on: bool);              // nSLEEP
    fn forward(&mut self, lane, duty);           // IN1_lane=PWM, ensure IN2=0
    fn reverse_all(&mut self, duty);             // IN1_*=0, IN2(GP6)=PWM
    fn brake_all(&mut self);                      // IN1_*=1, IN2=1
    fn coast(&mut self, lane);                    // IN1_lane=0
}
```

### 4.5 Bring-up notes

- **Direction polarity is per-motor and unknown until tested.** If a lane runs backward on
  "forward," swap that motor's two output leads at the terminal (a wiring fix). The TUNE
  mode (§6) includes a per-lane jog so you can verify direction during assembly.
- **Homing is gentle-stall:** motors reverse at a low homing duty (~40%) and briefly press
  against the end stop until *all* lanes have homed, then power is cut. Brief (<~1.5 s) and
  standard practice; keep homing duty low so the stall current/heat is small.
- Decouple each board's VM/GND with a **≥100 µF** bulk cap + 0.1 µF, and 0.1 µF across each
  motor's terminals (brush noise).

---

## 5. LED control — WS2812B, 5 columns, serpentine chain

### 5.1 Physical layout (top-down)

Four lanes `L0..L3` left→right; **5 LED columns `C0..C4`** run start→finish along the lane
boundaries. Lane `Li` is flanked by columns `Ci` (left) and `Ci+1` (right); interior
columns `C1/C2/C3` are **shared** by adjacent lanes. That's the "middle lanes share
columns" arrangement — 5 columns for 4 lanes.

### 5.2 Serpentine (boustrophedon) wiring — one data wire

```
Pico GP9 → 74AHCT125 → C0(start→finish) → C1(finish→start) → C2(start→finish)
                       → C3(finish→start) → C4(start→finish)
```

Data enters C0 at the **start**, runs to the finish, jumps to C1's **finish** end (C1 runs
backward), jumps to C2's **start** (forward), etc. **Even columns are wired forward, odd
columns reversed** — exactly your description, direction flipping each column.

- **74AHCT125 level shifter** on the GP9 data line (3.3 → 5 V) — required, don't skip.
- Inject 5 V at both ends of the chain if the far LEDs dim/shift color (chase draws little,
  so likely fine, but leave pads for it).

### 5.3 Count-per-column config & index mapping (`leds.rs`)

All columns are the same physical length, but keep the per-column count **configurable** so
trimming differences don't break the math and so `N` is a single source of truth:

```rust
pub const COUNTS: [usize; 5] = [18, 18, 18, 18, 18];   // ← measure & edit after build
pub const N: usize = COUNTS[0]+COUNTS[1]+COUNTS[2]+COUNTS[3]+COUNTS[4]; // PioWs2812<…,N,…>
const OFFSET: [usize;5] = /* prefix sums of COUNTS */;

/// Physical chain index for a column `c` and a position `p` measured FROM THE START.
/// Absorbs the serpentine direction flip so game code never sees raw indices.
fn phys_index(c: usize, p: usize) -> usize {
    if c % 2 == 0 { OFFSET[c] + p }                    // forward column
    else          { OFFSET[c] + (COUNTS[c]-1 - p) }    // reversed column
}

/// Map a lane progress f∈[0,1] (0 = start, 1 = finish) to a column position.
fn pos(c: usize, f: f32) -> usize { (f * (COUNTS[c]-1) as f32 + 0.5) as usize }
```

> **`N` is a compile-time const generic** for `PioWs2812<PIO0, 0, N, Grb>`. "Adjust in
> software for final count per column" = edit the `COUNTS` array and rebuild — one line,
> and all mapping/offsets/`N` follow automatically.

### 5.4 Rendering

Work entirely in **(lane, progress)** space; `phys_index` handles wiring direction.

- **Race chase** — for each lane `i` at progress `f_i`, draw a comet on *both* flanking
  columns: `set(phys_index(i, pos(i, f_i)))` and `set(phys_index(i+1, pos(i+1, f_i)))`
  (plus a short fading tail). Because a shared interior column borders two lanes, it can
  show up to **two** comets — composite into the framebuffer with a per-pixel **max/OR**.
- **Selecting** — light the selected lane's two columns (solid/dim pulse) in that duck's
  color.
- **Winner** — bright flash + chase on the winning lane's two columns.
- **Attract** — slow idle shimmer across all columns. Borrow the reference's pulse *idea*
  but **not its per-frame `powf`/`sinf` per channel** — use a **256-entry gamma LUT** and
  keep transcendentals out of the inner loop (§2.1). At ~90 LEDs the reference's approach
  would waste real cycles on the FPU-less M0+.
- Frame tick ~30 ms in `led_task`; game logic never blocks it (§7, §2.1).

Progress `f_i(t)` is **dead-reckoned from elapsed time × commanded speed** (no mid-track
sensor), clamped to 1 at the finish switch. The LED chase reflects *commanded* progress;
the *actual* winner is whoever trips the end switch first. Minor visual mismatch is fine.

---

## 6. Motor baseline tuning — on-the-fly, no hard-coding

TT gearmotors vary unit-to-unit (and drift as brushes wear), so each lane needs a baseline
duty trim. Two ways; **flash-calibration is the primary** (no extra parts, precise,
persistent).

### 6.1 Primary — calibration mode + flash persistence (`calibrate.rs`)

- **Enter:** hold **GO at power-up** → `Tune` state instead of the normal game.
- **In TUNE:**
  - Duck-select buttons pick the **active lane** (its column lights + shows a level bar).
  - **UP/DOWN** (GP15/16) nudge that lane's baseline duty in ~2% steps; the column bar
    reflects the level live.
  - **GO** runs a single **test race** on the active lane at its current baseline (forward
    to the end switch, then auto-home) so you can eyeball the speed.
  - **Hold GO** (long-press) → **save** all four baselines to flash and exit to the game.
- **Storage:** a small versioned struct `{ magic, version, baseline: [u16;4], crc }` in the
  **last flash sector** via `embassy_rp::flash` (or `sequential-storage` for wear-leveled
  KV). Load on boot; fall back to safe defaults if magic/CRC invalid. **Write only on
  explicit save** (flash wear).
- Final race duty per lane: `clamp(baseline_i + random_offset (+ optional mid-race surge),
  FLOOR, 100)` — baseline centers each lane so the race is fair; randomness adds the fun.

### 6.2 Alternative — physical trim knobs (if you'd rather have knobs)

Four pots read live = "knob position *is* the value," no flash needed. But the RP2040 has
only **3 usable ADC channels**, so four pots need an external ADC: **ADS1115** (I²C, 4×
single-ended, addr 0x48, ~$4) on 2 pins. Downsides vs flash-cal: knobs drift/get bumped,
less precise. Recommend flash-calibration unless you specifically want tactile knobs on the
panel. (GP28's spare ADC could still host one *global* speed pot if you like.)

---

## 7. Game loop & architecture (async)

Same shape as your reference: small tasks + a channel + a decoupled renderer.

```
[button/limit input tasks] --Event--> EVENTS channel --> game_task (FSM + Motors + tuning)
                                                            |
                                                     RACE_VIEW (Signal) --> led_task (PioWs2812)
watchdog fed inside game_task's loop
```

- **`inputs.rs`** — one `input_task` per button + limit (pool size ~16). Await falling
  edge, `Timer::after(25 ms)` debounce, re-check level, push a `Copy` `Event` into
  `EVENTS`. `Event = Select(u8) | Go | Up | Down | StartHit(u8) | EndHit(u8)`.
- **`game.rs`** — owns `Motors`, the FSM, and per-race randomization (`SmallRng` seeded
  from `RoscRng`). Publishes a `RaceView { progress:[f32;4], mode }` via
  `Signal<CriticalSectionRawMutex, RaceView>` for the renderer.
- **`led_task`** — owns `PioWs2812`; on a ~30 ms ticker reads `RACE_VIEW` and draws (§5.4).
- **`calibrate.rs`, `audio.rs`** — see §6, §9.
- Single-core executor is plenty. Feed the `Watchdog` (~8 s) at the top of the game loop.

### State machine

| State | Behavior | → next |
|-------|----------|--------|
| **Home** | `reverse_all(homing)`; wait until all 4 StartHit (per-lane `with_timeout`), then cut power | Attract |
| **Attract** | idle shimmer; wait for any `Select` | Selecting |
| **Selecting** | track current pick; `Select` updates it; light its lane | on `Go` (pick set) → Race |
| **Race** | randomize speeds from baselines, kick, drive forward, publish progress; await first `EndHit` (overall `with_timeout`) | on `EndHit(w)` → Winner(w); on timeout → Home |
| **Winner(w)** | `brake_all`; winner flash/chase; (future: sound); ~4 s | Home |
| **Tune** | calibration UX (§6); entered only via GO-held-at-boot | Home (on save) |

Randomization: per race, offset each lane by ±X% around its baseline; optionally give one
random lane a brief mid-race "surge" for drama. All clamped to `FLOOR..=100`.

Input gating: ignore `Select/Go` during Race/Home; ignore `StartHit/EndHit` when not
racing/homing.

---

## 8. `config.rs` — single source of truth

Centralize: pin assignments (mirrors §3), `COUNTS`/`N`, `PWM_TOP`, `DUTY_FLOOR`,
`DUTY_KICK`, `DUTY_HOMING`, timeouts (`RACE_TIMEOUT`, `RESET_TIMEOUT`), debounce, frame
interval, duck colors, `random_offset` spread. One place to tune the feel.

---

## 9. Audio — stub now, DFPlayer later (`audio.rs`)

No audio hardware yet. Define the interface and trigger points now; implement later.

- **Planned hardware:** DFPlayer Mini (MP3 + built-in amp) on **UART0 (GP0 TX / GP1 RX,
  9600 baud)**, microSD with clips, 3 W speaker, volume via DFPlayer command and/or a pot
  on **GP28** (ADC2 spare).
- **Trigger points (call sites wired now, no-op today):**
  - `Sound::Bet` — a duck is selected (Selecting).
  - `Sound::Race` — race start / looping race bed (Race entry).
  - `Sound::Finish` — first duck crosses the line (Winner entry).
  - (nice-to-have) `Sound::Attract`, `Sound::Home`.
- **Stub interface:**
  ```rust
  pub enum Sound { Bet, Race, Finish, Attract, Home }
  pub trait AudioSink { fn play(&mut self, s: Sound); fn set_volume(&mut self, v: u8); }
  pub struct NullAudio; // current: logs via defmt, does nothing
  // later: struct DfPlayer<UART> { … } building 10-byte 0x7E…0xEF command frames
  ```
  `game.rs` calls `audio.play(Sound::…)` at the trigger points regardless of backend, so
  adding the DFPlayer later is a drop-in swap of `NullAudio` → `DfPlayer`.

---

## 10. Other unknowns / things to consider

1. **Motor direction per lane** — unknown until wired; verify with TUNE jog, fix by
   swapping that motor's output leads. (§4.5)
2. **Finish overshoot / end stops** — put hard bumpers at both extremes so a gantry can't
   run off the extrusion; place the end switch just *before* the bumper; brake-on-finish
   reduces the slam.
3. **Inrush / brownout** — four motors launching together can dip the 5 V rail and reset
   the Pico. Mitigate with the VM bulk caps (§4.5), a **stable/separate 5 V branch for the
   Pico**, common ground, and optionally a few-ms **staggered launch** or soft ramp.
4. **Common ground** across Pico, both DRV8833s, motor supply, and LED supply — mandatory.
5. **WS2812 count is compile-time** — recompile after measuring final per-column counts
   (edit `COUNTS`). (§5.3)
6. **Level shifter** on WS2812 data (74AHCT125) — reiterating; #1 flaky-LED cause.
7. **Home-switch actuation** — the gantry must firmly trip a lever microswitch before the
   hard stop; mount with a little pre-travel.
8. **Tie / near-simultaneous finish** — first `EndHit` sampled wins; document as
   acceptable for a for-fun game (sub-ms differences are invisible anyway).
9. **Flash write stalls everything** — an erase/write pauses XIP and freezes all tasks
   (incl. animation) for tens of ms, and wears the sector. Write baselines **only on
   explicit save in TUNE mode**, never during a race. (§2.1, §6.1)
10. **Belt tension / pulley grub screws** — slip makes commanded speed ≠ actual speed;
    tension the loop and Loctite the grub screws, else re-tuning won't hold.
11. **Debug probe (rs-probe)** — SWD: SWCLK/SWDIO/GND to the Pico's 3-pin debug header;
    `cargo run --release` = `probe-rs run --chip RP2040` with live RTT logs.
12. **Re-tune over time** — brushes wear; persistent baselines + easy TUNE mode make
    periodic re-cal painless.
13. **Power budget / fuse** — motors (up to ~4 A peak inrush) + LEDs (chase ~1–2 A) on the
    5 V/10 A rail; inline fuse + panel switch.
14. **Boot safety** — home slowly on power-up; handle a duck already resting on a finish
    switch (don't count it as an instant win).

---

## 11. Bring-up — assemble one lane at a time (feature-gated test modes)

Bring-up follows the physical assembly order. Each mode is a cargo feature (enable exactly
one); the normal build is the full game. See `bringup.rs`.

**Do this first, before any motor jogging:** install the **home *and* finish bumpers**
(mechanical, independent of the switches). In `test-motors` there are no limit switches yet,
so nothing in software stops a gantry from running off the end of the extrusion — the
bumpers are the backstop. Use short taps until you trust the travel.

1. **`--features test-motors`** — motor on the track, **no switches needed**. Tap a duck
   button to pick the lane; **hold UP = forward, hold DOWN = reverse** at `JOG_DUTY_PCT`
   (edit in `config.rs`). Verify: motor spins, belt drives the gantry, and **"forward"
   moves toward the finish** — if reversed, swap that motor's two output leads at the
   terminal. (Reverse uses the shared line, so it drives every *connected* motor; fine
   while you have one lane wired.)
2. **`--features test-lane`** — add that lane's start+finish switches. Tap a duck button to
   pick the lane, **GO** = drive to the finish switch, then reverse home to the start
   switch. RTT logs the times; confirms both switches, direction, and homing. Repeat
   test-motors → test-lane per lane as you build each one.
3. **`--features test-leds`** — after all tracks are placed and the LED strips installed:
   Pass 1 walks a dot through the physical chain (watch it snake), Pass 2 lights each
   column a distinct colour, Pass 3 walks a dot from each column's **start** end. Use this
   to **verify wiring/direction and set the real `COUNTS`** in `config.rs`, then rebuild.
4. **Default build (full game)** — everything but audio is done. Power up, hold **GO at
   boot** to enter **TUNE** and calibrate each lane's baseline (saved to flash), then play.
5. **Audio** — keep `NullAudio`; drop in the DFPlayer backend when the hardware arrives.

> **Is this order optimal?** Yes — it matches your assembly and de-risks the drive train
> first (the highest-uncertainty part). Two refinements folded in above: install the
> bumpers *before* step 1 (no software end-stop exists yet), and do the forward-direction
> check in step 1 so a mis-wired motor is caught before switches are involved. Run the
> TUNE calibration (step 4) only once the whole rig is assembled, since baselines depend
> on the real belt tension and friction of the finished lanes.

---

## 12. Proposed file layout

```
firmware/
  Cargo.toml            # Embassy 0.9-era deps (§1)
  .cargo/config.toml    # probe-rs runner, target
  build.rs, memory.x    # RP2040 layout (from reference)
  src/
    main.rs             # init, bind_interrupts, spawn tasks, watchdog
    config.rs           # pins, COUNTS/N, duties, timings, colors (§8)
    motors.rs           # DRV8833 abstraction, shared-IN2 logic (§4)
    inputs.rs           # input_task, Event enum, debounce (§7)
    game.rs             # FSM, race logic, randomization (§7)
    leds.rs             # Layout, phys_index, animations, PioWs2812 (§5)
    calibrate.rs        # TUNE mode + flash load/save (§6)
    audio.rs            # Sound/AudioSink stub → DFPlayer later (§9)
    bringup.rs          # feature-gated motor-jog / single-lane test modes (§11)
```

All four build configurations (default + the three `test-*` features) compile clean for
`thumbv6m-none-eabi`. Untested on hardware — that's what §11 is for.
