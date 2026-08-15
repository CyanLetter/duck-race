# Duck Race — Firmware Implementation Plan

Target: **Raspberry Pi Pico (RP2040, plain — not W)**, Embassy async Rust, flashed/debugged
over a **Raspberry Pi Debug Probe (rs-probe)** with `probe-rs` + `defmt` RTT.

Game (**side-on cabinet** — you watch the ducks in profile, lanes stacked vertically):
player presses a duck-select button, presses GO, four ducks race down the V-slot lanes at
randomized speeds, first to trip its finish switch wins, the winner is shown on the lane
LEDs, then the machine re-homes. **No payout logic, no odds, no bets** — prizes are handed
out by hand at the convention. This keeps the state machine simple:
`select → race → show winner → reset`.

> This document is the plan. The existing `firmware/` scaffold (`Cargo.toml`, `main.rs`,
> `ws2812.rs`) predates it and the reference-project findings below — it will be
> **regenerated** to match this plan (Embassy 0.9, DRV8833, built-in `PioWs2812`) when we
> start building.

### Revision — hardware changes since first draft (2026-08-09)

Four decisions changed after the first lane's drive train was brought up. Sections below
are updated; this is the summary of what moved and what it costs in firmware.

| Change | Was | Now | Firmware impact |
|--------|-----|-----|-----------------|
| **Motors** | TT gearmotor ~200 RPM @ 6 V | **N20 6 V, 500 RPM** | Retune every duty constant (§4.3) — the whole race now lives in a **~25–50 %** duty band instead of ~50–75 %. Lower breakaway, lower stall current, gentler homing. |
| **Viewing** | Top-down (pinball) | **Side-on** (lanes stacked) | LEDs go from **5 shared columns → 4 independent rows**, one per lane (§5). Serpentine wiring unchanged. Mapping simplifies (no shared-column compositing). |
| **Audio** | DFPlayer Mini (SD, `7E…EF` frames) | **DY-SV8F** (onboard flash, `AA…` frames) | Still UART, still 2 pins, `AudioSink` trait unchanged — only the frame builder differs (§9). |
| **Pin map** | Ad-hoc, assigned as written | **Grouped in banks of four** aligned to Pico header runs | Every switch/button bank is now a solderable 4-signal + GND header (§3). Pure renumbering — no logic change. |

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
  - `cortex-m 0.7`, `cortex-m-rt 0.7`, `critical-section 1.1`, `portable-atomic 1.5`
    (with the `critical-section` feature — the M0+ has no native atomic CAS)
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
  destructure, RTT logging. (`static_cell` turned out to be unused here and was dropped —
  it pulled in an atomic-CAS dependency the M0+ can't satisfy.)

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
for our 72–144).

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
  path for the whole chain.
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

## 3. Pin map (RP2040) — grouped in banks of four

Reserved by the board: GP23/24 (SMPS/VBUS), GP25 (onboard LED), GP29 (VSYS). SWD debug
(SWCLK/SWDIO/GND) uses the dedicated 3-pin debug header, **not** GPIO. That leaves
**26 usable GPIO** (GP0–GP22, GP26–GP28) and the design needs 25 of them — this map is
tight on purpose, see §3.3 for the pressure valves.

### 3.1 What is actually pin-constrained (and what isn't)

Assign the constrained functions **first**, then shuffle the free ones for connector
convenience. This is the list to check before moving anything.

| Function | Constraint | Legal pins (Pico header only) |
|---|---|---|
| **Audio UART TX/RX** | must be a matched UART TX/RX pair | UART0: TX `0, 12, 16, 28` / RX `1, 13, 17` · UART1: TX `4, 8, 20` / RX `5, 9, 21` |
| **Motor PWM ×5** | any GPIO, but each needs a **distinct slice+channel** | `slice = (GP >> 1) & 7`, channel **A** if GP even / **B** if odd. **Pins 16 apart collide** (GP2 and GP18 are both slice1 A). |
| **WS2812 data** | any GPIO (PIO can map anywhere) | any — pick one physically next to a GND pad for the data return |
| **ADC** (only if you add a volume/trim pot) | hard-wired | **`26, 27, 28` only** |
| **I²C** (only if an expander is ever added) | matched SDA/SCL pair | I²C0 SDA `0,4,8,12,16,20,28` / SCL `1,5,9,13,17,21` · I²C1 SDA `2,6,10,14,18,22,26` / SCL `3,7,11,15,19,23,27` |
| Buttons, limit switches, nSLEEP, nFAULT | **none** — plain digital I/O | any GPIO |

Everything in that last row — 4 selects, 3 control buttons, 8 limit switches, nSLEEP,
nFAULT (15 of the 25 pins) — is free to place, which is what makes the banking below work.

### 3.2 Proposed map (banked for solderable headers)

The Pico's header has a **GND every 5th physical pin**, so four GPIO runs land as a clean
**4-signal + common-ground 5-pin connector**: `GP2–5`, `GP6–9`, `GP10–13`, `GP18–21`.
Those four runs are spent on the four things that come in fours.

| GPIO | Physical pin | Function | Connector |
|------|-----|----------|-----------|
| GP0  | 1  | Audio **UART0 TX** → DY-SV8F RX | **3-pin: audio** (pins 1–3, GND at 3) |
| GP1  | 2  | Audio **UART0 RX** ← DY-SV8F TX | ↑ |
| GP2  | 4  | Motor 0 IN1 — forward PWM | **5-pin: motor PWM bank** (pins 4–8, GND at 8) — PWM slice1 A |
| GP3  | 5  | Motor 1 IN1 — forward PWM | ↑ slice1 B |
| GP4  | 6  | Motor 2 IN1 — forward PWM | ↑ slice2 A |
| GP5  | 7  | Motor 3 IN1 — forward PWM | ↑ slice2 B |
| GP6  | 9  | **Finish** limit — lane 0 | **5-pin: finish-switch bank** (pins 9–13, GND at 13) |
| GP7  | 10 | Finish limit — lane 1 | ↑ |
| GP8  | 11 | Finish limit — lane 2 | ↑ |
| GP9  | 12 | Finish limit — lane 3 | ↑ |
| GP10 | 14 | Duck-select button 0 | **6-pin: arcade-button panel** (pins 14–19 = GP10, GP11, GP12, GP13, **GND**, GP14) |
| GP11 | 15 | Duck-select button 1 | ↑ |
| GP12 | 16 | Duck-select button 2 | ↑ |
| GP13 | 17 | Duck-select button 3 | ↑ |
| GP14 | 19 | **GO** (hold at boot → TUNE) | ↑ — same connector as the ducks, ground in the middle |
| GP15 | 20 | *spare* — 6th panel button / audio BUSY | extends the panel connector to 7-pin if used |
| GP16 | 21 | TUNE **UP / +** | **3-pin: TUNE** (pins 21–23, GND at 23) |
| GP17 | 22 | TUNE **DOWN / −** | ↑ |
| GP18 | 24 | **Home** (start) limit — lane 0 | **5-pin: home-switch bank** (pins 24–28, GND at 28) |
| GP19 | 25 | Home limit — lane 1 | ↑ |
| GP20 | 26 | Home limit — lane 2 | ↑ |
| GP21 | 27 | Home limit — lane 3 | ↑ |
| GP22 | 29 | WS2812 data → 74AHCT125 → LED chain | 1 wire (PIO0 SM0) |
| GP26 | 31 | **Shared** IN2 — reverse/home PWM, all 4 motors | **4-pin: driver control** (pins 31–34; AGND at 33 is the connector's ground) — PWM slice5 A |
| GP27 | 32 | DRV8833 nSLEEP (both boards) | ↑ |
| GP28 | 34 | DRV8833 nFAULT (both, wired-OR, pull-up) | ↑ |
| GP25 | — | Onboard LED — heartbeat/status | on-board |

**Why this arrangement:**
- **All five arcade buttons are one connector.** GP10–GP14 is a single physical run
  (pins 14–19) with GND at position 5, so the four ducks *and* GO leave the board on one
  6-way loom. GP15 is the spare immediately after GO, so a 6th panel button extends that
  same connector rather than starting a new one.
- **TUNE up/down sit alone on GP16/GP17** (pins 21–23 with GND at 23), entirely on the
  right-hand side. They're a service control, not part of the player panel, so giving them
  their own small header keeps the two looms independent.
- **Home switches and finish switches are separate banks of four** (`GP18–21`, `GP6–9`),
  so `Event::StartHit(n)` / `EndHit(n)` map to `bank_base + n` with no lookup table.
- **The five motor PWMs are split** deliberately: the four per-lane forwards get the tidy
  `GP2–5` bank (two slices, A/B pairs — same frequency, independent duty, exactly what we
  want), and the shared reverse moves to **GP26** so it doesn't eat a bank. GP26 is
  slice5 A — no collision with slices 1/2.
- **GP22 for LED data** sits alone next to GND pin 28; a single signal wire to the level
  shifter is all it needs.
- nSLEEP/nFAULT/reverse-PWM ride together on pins 31–34 as one 4-pin driver-control
  connector — those three are the only lines the DRV8833 pair needs besides the PWM bank.

**No split connectors.** Physical pins 20 and 21 are on *opposite sides* of the board —
that seam falls on GP15, the spare, so every functional group lands wholly on one side.
(An earlier draft put GO/UP/DOWN on GP14–GP16 and straddled it; this arrangement avoids
that.)

### 3.3 Pressure valves (all 26 GPIO are allocated)

There is exactly one true spare (**GP15**). If you need more pins:

1. **Drop audio RX (GP1)** — the DY-SV8F only needs TX for play/stop/volume; RX is for
   status queries we don't use. Frees 1 pin, and GP0/GP1 are also I²C0 if you'd rather
   have a bus there.
2. **Drop nFAULT (GP28)** — diagnostic only; it bought us the wiring diagnosis during
   bring-up but the game doesn't act on it. **Note this is also the only ADC pin left** —
   if you ever want a physical master-volume knob, it takes GP28 and nFAULT goes.
3. **Audio one-line mode** — the DY-SV8F's single-bus mode needs 1 GPIO on *any* pin
   instead of a UART pair (§9). Costs flexibility (no volume command), frees a pin.
   Related: GP16/GP17 are the header's *other* UART0 TX/RX pair and are now spent on TUNE,
   so after this map there is no second UART pair free. Only matters if a serial
   peripheral beyond the audio module ever appears.
4. **MCP23017 I²C expander** for the 8 limit switches — frees 6 pins for 2. **Not
   recommended here:** the finish switches decide the winner, and putting them behind a
   polled I²C bus adds milliseconds of jitter to exactly the measurement that matters.
   If you ever need this, move the *home* switches (timing-insensitive) and keep the
   finish switches on native GPIO.

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
| Board A | 0 | GP2 | GP26 (shared) | AOUT1/2 → lane-0 motor |
| Board A | 1 | GP3 | GP26 (shared) | BOUT1/2 → lane-1 motor |
| Board B | 2 | GP4 | GP26 (shared) | AOUT1/2 → lane-2 motor |
| Board B | 3 | GP5 | GP26 (shared) | BOUT1/2 → lane-3 motor |

Both boards: **nSLEEP → GP27** (high = enabled), **VM → 5 V motor rail**, **GND → common
ground**, **nFAULT → GP28** (open-drain, one pull-up, wired-OR).

> **Board silkscreen, learned the hard way during bring-up:** on the common cheap DRV8833
> breakouts the enable and fault pins are labelled **`EEP` = nSLEEP** ("sl**EEP**") and
> **`ULT` = nFAULT** ("fa**ULT**"). `EEP` **must** be pulled high or the driver stays
> asleep and nothing moves. Also: one motor uses **one channel** — `IN1`+`IN2` → `OUT1`+
> `OUT2` (or `IN3`+`IN4` → `OUT3`+`OUT4`). Splitting a motor across two channels
> (IN1 + IN4) silently does nothing.

- **Race:** IN2 (GP26) = 0; each IN1 = PWM(speed_i) → all forward, independent speeds.
- **Home/reset:** all IN1 = 0; IN2 (GP26) = PWM(homing) → all reverse together.
- **Stop at finish:** all IN1 = 1 **and** GP26 = 1 → brake all (race is over for everyone,
  so braking the shared line is exactly right — stops the slam into the end bumper).

**Constraint this imposes:** you can't drive two lanes in *opposite* directions at once.
That never happens in this game, but it's why homing is "all-together" (§7).

### 4.3 PWM setup — retuned for **N20 6 V / 500 RPM** motors

- GP2/GP3 = slice1 A/B, GP4/GP5 = slice2 A/B, GP26 = slice5 A. A/B channels on a slice
  share frequency (good — we want one motor PWM frequency) with **independent compare**
  (independent duty). `Config.top = 6249` at 125 MHz / divider 1 ≈ **20 kHz** (above
  audible) so the motors don't whine; duty = `top × pct/100`.

**The speed math changed a lot.** With a 20 T GT2 pulley the belt moves `20 × 2 mm =
40 mm` per motor revolution:

| | TT (old) | **N20 500 RPM (new)** |
|---|---|---|
| No-load RPM @ 6 V | ~200 | **500** |
| Derated on the 5 V rail | ~165 | **~415** |
| Top belt speed @ 5 V, 100 % duty | ~110 mm/s (4.3 in/s) | **~277 mm/s (10.9 in/s)** |
| Duty needed for the 4 in/s target (≈102 mm/s) | ~90 % | **~36 %** |
| 610 mm race at that duty | ~5.5 s | **~6.0 s** |
| Full-tilt 610 mm run | ~5.5 s | **~2.2 s** (far too fast to be a race) |

So the **entire game now lives in a ~25–50 % duty band** instead of ~50–100 %. That is the
single biggest firmware consequence of the motor swap, and every duty constant moves:

| Constant | Was | **Now** | Why |
|---|---|---|---|
| `BASE_DEFAULT_PCT` | 60 | **36** | lands ~4 in/s → ~6 s race over 610 mm |
| `FLOOR_PCT` | 35 | **18** | N20 breakaway is much lower than a TT's; the floor must sit well below `base − spread` (36 − 4 = 32) or the clamp eats the slow ducks |
| `KICK_PCT` / `KICK_MS` | 90 / 120 | **55 / 60** | a 90 % kick is now a ~11 in/s lurch that eats 5 % of the track in the first frame. Rule of thumb: kick ≈ 1.5× base, briefly |
| `HOMING_PCT` | 35 | **40** | 35 % is only ~97 mm/s now → 6.3 s to home a full lane. 40 % ≈ 111 mm/s. Still gentle: N20 stall torque is low, so the press into the bumper is *softer* than the TT's despite the higher duty |
| `RESET_TIMEOUT_MS` | 8 000 | **10 000** | worst case is homing from the finish line at `HOMING_PCT` |
| `SPEED_SPREAD_PCT` | 12 (duty points) | **20 (percent of baseline)** | **semantics changed** — see below. The old ±12 on a base of 60 was ±20 % relative; kept as ±20 %, which is ≈ ±7 points at the new base of 36. Leaving it at 12 *points* would have meant ±33 % relative → 8.9 s vs 4.5 s finishes, a blowout every race |
| `NOMINAL_SECS` | 5.0 | **6.0** | keeps the LED dead-reckoning matched to the new base duty |
| `JOG_DUTY_PCT` | 100 | **35** ⚠️ | **safety**: `test-motors` has no software end stop, and 100 % is now ~11 in/s. A full lane traverses in 2.2 s |

> **Done — `SPEED_SPREAD_PCT` is now relative.** In absolute duty points it had to be
> re-derived every time the baseline moved, which is exactly what the motor swap forced.
> It is now a percentage **of each lane's own calibrated baseline**
> (`spread = base × pct/100`, applied as `base ± rand(spread)` in `game.rs::race`), so it
> survives motor swaps and per-lane calibration untouched. At base 36 and 20 %, race
> duties land in **29–43 %** ≈ 5.0–7.5 s finishes — well clear of `FLOOR_PCT`.

- **PWM floor**: DC motors won't creep smoothly near 0. Never command a moving motor below
  `FLOOR_PCT`. The N20 lowers this bar but doesn't remove it — confirm the real number per
  lane during TUNE (drive down until the lane stops moving reliably, add ~5 points).
- **Launch kick**: brief high duty at race start to break static friction, then settle to
  the lane's target duty.

### 4.3.1 What the N20 swap buys, and what it costs

**Buys:**
- **Lower breakaway voltage** — the exact failure that stalled bring-up at 55 % duty on
  the TT is much less likely; low-duty starts become reliable.
- **Much finer speed control.** At `top = 6249` the 25–50 % band is still ~1 560 compare
  steps, so resolution is a non-issue, and being *far* from the motor's top speed means
  duty→speed is more linear than a TT near its ceiling. Randomized per-race speeds and
  mid-race surges have real room to work now.
- **Lower stall current** (~0.6–1.5 A vs the TT's ~1.2 A+) → more DRV8833 headroom, less
  inrush on the shared 5 V rail, and a gentler stall-into-bumper during homing.

**Costs / things to verify:**
- **Torque is the trade for speed.** A 500 RPM N20 is at the low-torque end of the family
  (~0.15–0.3 kg·cm ≈ 3 N at a 20 T pulley's 6.4 mm radius ≈ 300 gf of belt pull). That
  should comfortably roll a V-wheel gantry, but it's the thing to check first — see the
  §11 bring-up note.
- **If 500 RPM proves twitchy or weak, a 300 RPM N20 is the better match.** It would put
  the target speed at ~60 % duty (a much more comfortable operating point) *and* give
  ~1.7× the torque, for the same money. Worth knowing before ordering three more.
- **Mechanical fit is different:** the N20 has a **3 mm D-shaft**, not the TT's double-D
  flat — you need **3 mm-bore** GT2 20 T pulleys and small N20 brackets (M2 hardware), not
  the TT-sized parts in the original BOM.
- **Small gearbox, repeated stalls.** Homing presses into a bumper every cycle. Keep
  `HOMING_PCT` low and the stall short (it already cuts power the moment the last lane
  homes) — this matters more on an N20 gearbox than a TT's.

### 4.4 Motor abstraction (`motors.rs`)

```
struct Motors { /* 3 Pwm slice handles, nSLEEP Output, shared config */ }
impl Motors {
    fn enable(&mut self, on: bool);              // nSLEEP
    fn forward(&mut self, lane, duty);           // IN1_lane=PWM, ensure IN2=0
    fn reverse_all(&mut self, duty);             // IN1_*=0, IN2(GP26)=PWM
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

## 5. LED control — WS2812B, **4 rows** (one per lane), serpentine chain

> **Changed with the move to a side-on cabinet.** The top-down plan used **5 columns for
> 4 lanes**, with the three interior columns shared between adjacent lanes. Side-on, each
> lane is a horizontal band you view in profile, so it gets **its own row**: **4 rows,
> 4 lanes, 1:1, nothing shared.** Serpentine wiring is unchanged. This is a simplification
> in every direction — fewer LEDs, less current, no shared-pixel compositing, and
> `lane == row` so the mapping layer nearly disappears.

### 5.1 Physical layout (side-on)

Four lanes `L0..L3` **stacked vertically** (`L0` at the top or bottom — pick one and keep
it consistent, see the note below); each lane gets **one LED row `R0..R3`** running
horizontally **start → finish**, mounted at the back of the lane so the duck doesn't
occlude it. Row `Ri` belongs to lane `Li` and nothing else.

> **Order matters now in a way it didn't top-down.** With rows stacked vertically, the
> chain order must match the physical top-to-bottom lane order, *and* both must match the
> duck-select button order on the panel. Get this backwards and duck 0's button lights
> duck 3's row. Verify it in `test-leds` (§11) before the panel is wired.

### 5.2 Serpentine (boustrophedon) wiring — one data wire

```
Pico GP22 → 74AHCT125 → R0(start→finish) → R1(finish→start)
                        → R2(start→finish) → R3(finish→start)
```

Data enters R0 at the **start**, runs to the finish, jumps down to R1's **finish** end (R1
runs backward), jumps to R2's **start** (forward), etc. **Even rows are wired forward, odd
rows reversed** — the same flip-every-row scheme as before, just 4 runs instead of 5.

- **74AHCT125 level shifter** on the GP22 data line (3.3 → 5 V) — required, don't skip.
- Inject 5 V at both ends of the chain if the far LEDs dim/shift colour (a chase draws
  little, so likely fine, but leave pads for it).

### 5.3 Count-per-row config & index mapping (`leds.rs`)

All rows are the same physical length, but keep the per-row count **configurable** so
trimming differences don't break the math and so `N` is a single source of truth:

```rust
pub const COUNTS: [usize; 4] = [18, 18, 18, 18];       // ← measure & edit after build
pub const N: usize = COUNTS[0]+COUNTS[1]+COUNTS[2]+COUNTS[3];   // PioWs2812<…,N,…>
const OFFSET: [usize;4] = /* prefix sums of COUNTS */;

/// Physical chain index for row `r` at position `p` measured FROM THE START.
/// Absorbs the serpentine direction flip so game code never sees raw indices.
fn phys_index(r: usize, p: usize) -> usize {
    if r % 2 == 0 { OFFSET[r] + p }                    // forward-wired row
    else          { OFFSET[r] + (COUNTS[r]-1 - p) }    // reversed row
}

/// Map a lane progress f∈[0,1] (0 = start, 1 = finish) to a position in that lane's row.
fn pos(r: usize, f: f32) -> usize { (f * (COUNTS[r]-1) as f32 + 0.5) as usize }
```

> **`N` is a compile-time const generic** for `PioWs2812<PIO0, 0, N, Grb>`. "Adjust in
> software for the final count per row" = edit the `COUNTS` array and rebuild — one line,
> and all mapping/offsets/`N` follow automatically.

**Density is now worth reconsidering.** With one row per lane instead of two flanking
flanking columns, that row is the *only* readout of the duck's position, so smoothness
matters more:

| Strip | Per 610 mm row | `N` total | Position resolution | Attract-mode current (all lit, 35 %) |
|---|---|---|---|---|
| 30 LEDs/m | 18 | **72** | 34 mm/pixel | ~1.5 A |
| 60 LEDs/m | 36 | **144** | 17 mm/pixel | ~3.0 A |

Either fits the 5 V/10 A budget and the RP2040 (144 × 3 B = 432 B of framebuffer). **The
plan defaults to `[18; 4]` (72 total, down from 90)** — bump to `[36; 4]` if the comet
looks steppy once assembled, and cap attract-mode brightness either way.

### 5.4 Rendering

Work entirely in **(lane, progress)** space; `phys_index` handles wiring direction.

- **Race chase** — for each lane `i` at progress `f_i`, draw a comet with a short fading
  tail at `phys_index(i, pos(i, f_i))`. **One row, one comet** — the per-pixel `max`
  compositing that existed to merge two comets on a shared interior column is no longer
  needed for that purpose (keep `max` only for a comet's own overlapping tail).
- **Selecting** — light the selected lane's row (solid/dim pulse) in that duck's colour,
  others dim.
- **Winner** — bright flash + chase along the winning lane's row.
- **Attract** — slow idle shimmer across all rows. Borrow the reference's pulse *idea* but
  **not its per-frame `powf`/`sinf` per channel** — use a **256-entry gamma LUT** and keep
  transcendentals out of the inner loop (§2.1).
- Frame tick ~30 ms in `led_task`; game logic never blocks it (§7, §2.1).

> **Side-on opens up an effect that top-down couldn't do:** because each lane owns a full
> row, the row can double as a **per-lane progress bar** (fill behind the duck) rather than
> just a comet — reads clearly in profile from across a convention hall. Same data, one
> branch in `render`. Worth trying once the strips are up.

Progress `f_i(t)` is **dead-reckoned from elapsed time × commanded speed** (no mid-track
sensor), clamped to 1 at the finish switch. The LED chase reflects *commanded* progress;
the *actual* winner is whoever trips the end switch first. Minor visual mismatch is fine.

---

## 6. Motor baseline tuning — on-the-fly, no hard-coding

Small brushed gearmotors vary unit-to-unit (and drift as brushes wear), so each lane needs
a baseline duty trim. This matters *more* with the N20s, not less: the game now runs in a
narrow ~25–50 % duty band (§4.3), so a couple of points of lane-to-lane difference is a
larger fraction of the commanded speed than it was with the TTs. Two ways;
**flash-calibration is the primary** (no extra parts, precise, persistent).

### 6.1 Primary — calibration mode + flash persistence (`calibrate.rs`)

- **Enter:** hold **GO at power-up** → `Tune` state instead of the normal game.
- **In TUNE:**
  - Duck-select buttons pick the **active lane** (its row lights + shows a level bar).
  - **UP/DOWN** (GP16/17) nudge that lane's baseline duty in ~2% steps; the row bar
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
only **3 usable ADC channels** (GP26/27/28), so four pots need an external ADC:
**ADS1115** (I²C, 4× single-ended, addr 0x48, ~$4) on 2 pins. Downsides vs flash-cal:
knobs drift/get bumped, less precise. Recommend flash-calibration unless you specifically
want tactile knobs on the panel.

> **New constraint under the §3.2 pin map:** all three ADC pins are now spent on driver
> control (GP26 reverse PWM, GP27 nSLEEP, GP28 nFAULT). Any pot — trim *or* master volume
> — costs you nFAULT (GP28), which is the intended trade (§3.3). Volume is a UART command
> on the DY-SV8F anyway (§9), so a volume pot isn't needed.

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

Centralize: pin assignments (mirrors §3), `COUNTS`/`N`, `PWM_TOP`, `FLOOR_PCT`,
`KICK_PCT`/`KICK_MS`, `HOMING_PCT`, `BASE_DEFAULT_PCT`, `SPEED_SPREAD_PCT`, timeouts
(`RACE_TIMEOUT_MS`, `RESET_TIMEOUT_MS`), debounce, frame interval, duck colours. One place
to tune the feel.

**The N20 / side-on retune, as applied:**

```rust
pub const ROWS: usize = LANES;                    // new — 4 rows, lane == row  (§5)
pub const COUNTS: [usize; 5] = [18,18,18,18,18];  →  [usize; ROWS] = [18,18,18,18]
pub const BASE_DEFAULT_PCT:  u8 = 60;  →  36      // §4.3
pub const FLOOR_PCT:         u8 = 35;  →  18
pub const KICK_PCT:          u8 = 90;  →  55
pub const KICK_MS:          u64 = 120; →  60
pub const HOMING_PCT:        u8 = 35;  →  40
pub const SPEED_SPREAD_PCT:  u8 = 12;  →  20      // ⚠ semantics: now % OF BASELINE, not duty points
pub const NOMINAL_SECS:     f32 = 5.0; →  6.0
pub const RESET_TIMEOUT_MS: u64 = 8_000; → 10_000
pub const JOG_DUTY_PCT:      u8 = 100; →  35      // ⚠ safety: no software end stop in test-motors
```

`PWM_TOP` (6249 ≈ 20 kHz), `RACE_TIMEOUT_MS`, `WINNER_SHOW_MS`, `DEBOUNCE_MS`, `FRAME_MS`
and `WATCHDOG_MS` are unaffected by the hardware changes.

---

## 9. Audio — stub now, **DY-SV8F** later (`audio.rs`)

> **Changed from DFPlayer Mini to the DY-SV8F.** (Sold as "DV-SV8F" by some resellers; it's
> the DY-series module.) Good news first: **nothing structural changes.** It's still a
> serial-controlled MP3 module with a built-in amp on **2 UART pins at 9600 baud**, the
> `AudioSink` trait is untouched, and the game's trigger points are identical. Only the
> **command frame format** differs, which is entirely inside the backend struct.

- **Hardware:** DY-SV8F on **UART0 — GP0 TX → module RX, GP1 RX ← module TX** (§3.2, 3-pin
  header with GND at physical pin 3), 4 Ω/8 Ω speaker on the module's built-in amp, volume
  set by serial command (no pot needed — which is good, since all three ADC pins are spent;
  §6.2).
- **Onboard 4 MB flash, no microSD.** Clips are loaded over USB (the module enumerates as
  mass storage). **Track index = the order files were written to flash**, not alphabetical
  — copy them **one at a time, in the order you want them numbered**, or track selection
  will be scrambled. This is the classic DY-series gotcha.
- **Mode selection is a hardware config, not software.** The DY-SV8F picks UART vs one-line
  vs IO-combination mode from the strapping on its `CON`/IO pins **at power-up**. You've
  said it's set for UART — worth confirming the strap resistors are actually populated
  before writing any driver code, because in IO mode the UART simply won't answer.
- **Protocol differs from the DFPlayer** — this is the only real code delta:

  | | DFPlayer Mini (old plan) | **DY-SV8F (now)** |
  |---|---|---|
  | Frame | 10 bytes, `7E FF 06 CMD 00 P1 P2 CK CK EF` | variable, `AA <CMD> <LEN> <DATA…> <SUM>` |
  | Checksum | 16-bit two's complement | **low byte of the additive sum** of all preceding bytes |
  | Typical ops | play / pause / volume / track | play / pause / stop / volume / select track |

  Take the **opcodes from the module's own datasheet** rather than from this document —
  the DY series has several variants and the command tables differ between them. The frame
  *shape* above is what the code should be built around; write a small
  `fn frame(cmd: u8, data: &[u8]) -> heapless::Vec<u8, 8>` helper that appends the additive
  checksum, and every command becomes one line.
- **3.3 V drive:** the module runs off 5 V but its serial input is 3.3 V-compatible. Put a
  ~1 kΩ series resistor on the Pico's TX line anyway, and decouple the module's 5 V — the
  built-in amp pulls real current on transients and shares a rail with the motors.
- **Optional `BUSY` line** → **GP17**, the one spare pin (§3.3), if you want the game to
  know when a clip has finished rather than timing it open-loop.
- **Trigger points (call sites wired now, no-op today):**
  - `Sound::Bet` — a duck is selected (Selecting).
  - `Sound::Race` — race start / looping race bed (Race entry).
  - `Sound::Finish` — first duck crosses the line (Winner entry).
  - (nice-to-have) `Sound::Attract`, `Sound::Home`.
- **Stub interface (unchanged):**
  ```rust
  pub enum Sound { Bet, Race, Finish, Attract, Home }
  pub trait AudioSink { fn play(&mut self, s: Sound); fn set_volume(&mut self, v: u8); }
  pub struct NullAudio; // current: logs via defmt, does nothing
  // later: struct DySv8f<UART> { … } building AA-prefixed, additive-checksum frames
  ```
  `game.rs` calls `audio.play(Sound::…)` at the trigger points regardless of backend, so
  adding the module later is a drop-in swap of `NullAudio` → `DySv8f`.

> **Pin-pressure fallback:** the DY-SV8F's **one-line single-bus mode** drives playback
> with timed pulses on a *single* GPIO (any pin), freeing one. It can only select tracks —
> no volume command — so it's a fallback, not the plan. **IO-combination mode is worse
> here**: it burns several GPIO to select clips by pin pattern, which is exactly the
> resource we're short of.

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
5. **WS2812 count is compile-time** — recompile after measuring final per-row counts
   (edit `COUNTS`, now `[usize; 4]`). (§5.3)
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

### New with the N20 / side-on / DY-SV8F changes

15. **N20 torque at 500 RPM is the open question.** ~0.15–0.3 kg·cm ≈ ~300 gf of belt pull
    at a 20 T pulley. Should roll a V-wheel gantry easily, but verify under the real belt
    tension before ordering three more. If it's marginal or the low-duty band feels
    twitchy, **300 RPM N20s are the better match** — more torque *and* a more comfortable
    ~60 % operating duty. (§4.3.1)
16. **N20 mechanical fit differs from the TT** — 3 mm D-shaft means **3 mm-bore GT2 20 T
    pulleys** and N20 brackets (M2), not the TT-sized parts in the original BOM. Check the
    parts on hand before assembly day.
17. **The duty band moved down, so `FLOOR_PCT` is now a live question, not a formality.**
    18 % is an estimate. Measure the real minimum-reliable-moving duty per lane during
    TUNE and set the floor ~5 points above it. Too high and it clamps away the slow ducks
    (base 36 − spread 4 = 32); too low and a lane can stall mid-race.
18. **Lane ↔ row ↔ button ordering** — side-on stacks the rows vertically, so the LED
    chain order, the physical top-to-bottom lane order, and the duck-button order must all
    agree. Verify in `test-leds` before wiring the panel. (§5.1)
19. **Occlusion, side-on** — mount the strips at the back of each lane so the duck body
    doesn't block its own progress indicator. Top-down never had this problem.
20. **Finish line reads edge-on now** — with lanes stacked, the four finish points are
    vertically aligned and hard to read at a glance from the side. Consider a physical
    finish-line marker or a distinct end-of-row LED treatment.
21. **DY-SV8F track order = flash write order**, not filename order. Copy clips one at a
    time in the intended sequence. (§9)
22. **DY-SV8F mode strapping** is latched at power-up from its CON/IO pins — confirm the
    UART-mode straps are populated before writing driver code. (§9)
23. **All 26 GPIO are allocated** under the §3.2 map, with GP17 the only spare. Any new
    peripheral means spending a pressure valve (§3.3) — decide before adding anything, not
    after.

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

   > ⚠️ **Drop `JOG_DUTY_PCT` to ~35 before jogging an N20 lane.** It was raised to 100
   > while chasing the TT's breakaway problem; at 500 RPM that's ~11 in/s and a full 610 mm
   > lane goes by in **2.2 seconds** with no software end stop. Short taps, bumpers in.
   >
   > **Also do the torque check here, while it's cheap (§10.15):** with the belt tensioned
   > and the gantry loaded as it will actually run, walk the duty *down* from 35 % and note
   > where the lane (a) still starts from rest reliably and (b) stops moving at all. Those
   > two numbers are your real `FLOOR_PCT` and the evidence for whether 500 RPM N20s are
   > the right motor before you buy three more.
2. **`--features test-lane`** — add that lane's start+finish switches. Tap a duck button to
   pick the lane, **GO** = drive to the finish switch, then reverse home to the start
   switch. RTT logs the times; confirms both switches, direction, and homing. Repeat
   test-motors → test-lane per lane as you build each one.
3. **`--features test-leds`** — after all tracks are placed and the LED strips installed:
   Pass 1 walks a dot through the physical chain (watch it snake), Pass 2 lights each
   **row** a distinct colour, Pass 3 walks a dot from each row's **start** end. Use this to
   **verify wiring/direction and set the real `COUNTS`** in `config.rs`, then rebuild.
   Side-on adds one thing to check here: **row 0 must be the same lane as duck button 0**
   — confirm the vertical order before the panel loom is soldered (§5.1, §10.18).
4. **Default build (full game)** — everything but audio is done. Power up, hold **GO at
   boot** to enter **TUNE** and calibrate each lane's baseline (saved to flash), then play.
   With the N20s this pass matters more than it did: the narrow duty band means the
   baselines carry more of the fairness (§6).
5. **Audio** — keep `NullAudio`; drop in the `DySv8f` backend when the module is strapped
   for UART and the clips are loaded in order (§9).

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
    audio.rs            # Sound/AudioSink stub → DY-SV8F later (§9)
    bringup.rs          # feature-gated motor-jog / single-lane test modes (§11)
```

All four build configurations (default + the three `test-*` features) compile clean for
`thumbv6m-none-eabi`. Untested on hardware — that's what §11 is for.
