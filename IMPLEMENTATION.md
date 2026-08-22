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
- **Dedicated `led_task` on a `Ticker`**, fed only the current `Mode` — decoupled from game
  logic (§7).
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
| `NOMINAL_SECS` | 5.0 | *(removed)* | only ever scaled the LED position dead-reckoning, which the marquee chase replaced (§5.4) |
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
plan defaulted to `[18; 4]`; the build shipped with **60 LEDs/m, `[39; 4]` = 156 total**,
which also gives the marquee chase (§5.5) enough pixels for a convincing bulb spacing.

### 5.4 Rendering

Work entirely in **(lane, position)** space; `phys_index` handles wiring direction.

- **Race** — the **carnival marquee chase** (§5.5). Deliberately **not** a position
  readout: it does not track where any duck actually is.
- **Attract** — the same chase, slower (`CHASE_STEP_MS_ATTRACT` vs `..._RACE`).
- **Selecting** — light the selected lane's row (pulsing) in that duck's colour, others
  dim. This is where lane identity matters, so it stays duck-coloured.
- **Winner** — bright blinking fill on the winning lane's row.
- **Home** — dim amber breathing across all rows.
- Frame tick ~30 ms in `led_task`; game logic never blocks it (§7, §2.1).

> **Dropped: per-lane position tracking.** The race used to dead-reckon each duck's
> progress from elapsed time × commanded duty and draw a comet at that point. It's gone,
> along with `RaceView.progress`, `draw_comet` and `pos`. Two reasons: in a side-on cabinet
> the duck itself is plainly visible, so a synthetic position readout added nothing; and
> once speeds began varying mid-race (§7.3) the dead-reckoning had to be integrated per
> frame to stay honest, which was real work to keep a display nobody was reading. The
> render signal is now just `Mode`.

### 5.5 The marquee chase — old incandescent carnival lights

Every row runs the **same chase, in phase**, so the whole board reads as one sign.

- A bulb lights at **every `CHASE_SPACING`-th pixel** (default 5) and the whole set
  advances **one pixel per `step_ms`**, travelling start → finish.
- Each bulb **snaps to full and then decays exponentially**. That asymmetry — instant on,
  slow off — is the entire trick: it's what a hot filament does when the current stops, and
  it's what separates "carnival" from "LED strip".
- Decay is `exp(-age / CHASE_TAU_STEPS)` where `age` is measured in **pixel-steps since
  this pixel was last the bulb**. At τ = 1.6 a bulb is at ~54 % one step later and ~8 %
  four steps later, so a tail has just faded out as the next bulb arrives — bulbs never
  fully extinguish, which is also true of the real thing at speed.

**Two implementation notes that matter:**

1. **It's stateless.** Brightness is a pure function of `(phase − position) mod spacing`,
   so there's no per-pixel decay buffer that has to be stepped in lockstep with the frame
   rate. Frame jitter can't corrupt the pattern, and the animation is fully determined by
   the clock.
2. **The decay curve is a LUT** (`CHASE_LUT_LEN` = 128 entries, built once at
   construction). Calling `expf` per pixel per frame would be the same soft-float mistake
   the reference project made with `powf` (§2.1) — 156 pixels × 33 fps of transcendentals
   on an FPU-less M0+.

`phase` is fractional, not integer, so the fade is smooth *between* steps rather than the
whole pattern jumping once per step.

**Colour:** `CHASE_COLOR`, a warm white (pre-gamma `255,180,100`) that lands near 2700 K
once the gamma LUT is applied. To chase in each lane's own duck colour instead, pass
`DUCK_COLORS[row]` inside `draw_chase` — one line.

**Tuning:** `CHASE_SPACING` = bulb density; `CHASE_STEP_MS_*` = speed (lower is faster);
`CHASE_TAU_STEPS` = tail length. Note spacing need not divide the row length — 39 pixels
with spacing 5 is fine and looks the way a real marquee does.

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
- The baseline is the *centre* each lane's mid-race speed segments are rolled around
  (§7.3) — calibration makes the lanes fair, the segment rolls add the fun.

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

  **Limit switches also publish a level cache** (`home_closed(lane)` /
  `end_closed(lane)`), seeded from the pin's real state when the task starts and kept in
  step on both edges. This exists because events are edge-triggered, which makes "the
  switch is *already* closed" invisible — see §7.1. `drain()` empties the queue for
  phases that must not act on a stale edge.

### 7.1 Edges vs. levels — which to use where

An early hardware bug: with a gantry parked on its home switch at power-up, `home()`
waited for a `StartHit` that could never arrive (no falling edge is generated by a switch
that was already closed before the task started), so **every boot in that position burned
the full `RESET_TIMEOUT_MS`**. The rule that came out of it:

| Question | Use | Why |
|---|---|---|
| "Is this lane home?" (homing, TUNE test run, `test-lane`) | **level** — `home_closed(l)` | The answer must be right when the switch was already closed. Correctness beats latency; a 50 ms poll is nothing against a multi-second homing move. |
| "Who crossed the line first?" (race) | **edge** — `EndHit` from `EVENTS` | The only measurement where timing resolution matters. An edge lands as soon as debounce clears, rather than at the next poll tick. |

The cost of the edge path is staleness, so `race()` calls `inputs::drain()` first — a
leftover `EndHit` sitting in the queue would otherwise score an instant false win.

**Two follow-on bugs in the level cache itself**, both found when *all four* lanes were
parked on their home switches at power-up (three-of-four had masked them):

1. **The level latched and never cleared.** `input_task` waited on a falling *edge*, then
   debounced, then waited for release. A switch already closed at boot never produces that
   falling edge, so the task parked on the first wait forever — and the `store(false)` on
   release sat downstream of it, unreachable. Once seeded `true` the level stayed `true`
   even after the gantry left home, so a later `home()` would take the "all lanes already
   home" path and skip homing entirely. **Fix:** the task now waits on **levels**, always
   for the *opposite* of where the pin currently is (`wait_for_high`/`wait_for_low` return
   immediately when already satisfied), so it always gets to run again on a change and
   writes the settled state every time.
2. **The seed raced task scheduling.** The cache was written on the task's first poll, so
   whether the first `home()` saw reality depended on something happening to `.await`
   between spawning and homing — true only by accident, and silently broken by reordering
   `main`. **Fix:** `inputs::spawn_input()` seeds the cache **synchronously at spawn time**,
   before any await. `main` also now constructs every input pin, waits 50 ms for the
   pull-ups to charge, and only then samples — a premature read on an open switch looks
   exactly like a closed one.

`home()` logs the raw switch state on every entry (`home: switch closed = [..]`), which
separates "the cache is wrong" from "the switch isn't physically closing" in one glance.

**Considered and rejected as a *detection* fix: nudging the gantry forward at boot** to
force a fresh edge. It treats the symptom, moves the machine before its state is known,
and leaves the same hang everywhere else that waits on a limit switch. (A boot nudge does
now exist — but for a genuine mechanical reason, not to paper over detection. See §7.1.1.)

#### 7.1.1 Boot nudge — a mechanical remedy, not a detection one

With detection correct, one physical failure remained: a gantry can come to rest **just
shy of its home switch's trip point** — close enough to look parked, not close enough to
close the contact — and reversing into an already seated gantry does not reliably move it
that last fraction of a millimetre.

So `run()` performs a **one-shot forward nudge before the first `home()`**
(`BOOT_NUDGE_PCT` for `BOOT_NUDGE_MS`, default 85 % for 200 ms ≈ 2 in). Every lane backs
off the home region, and the homing pass that follows arrives with momentum and seats the
switch cleanly. `BOOT_NUDGE_MS = 0` disables it.

Two details that matter:

- **Lanes already on their finish switch are skipped**, so a gantry parked at the far end
  can't be driven into the finish bumper. Forward is per-lane (each motor owns its IN1),
  so skipping one lane is free — unlike reverse, which is shared (§4.2).
- **It waits ~3 × `DEBOUNCE_MS` afterwards** before returning. The nudge *opens* the home
  switches, and the level cache only clears once each input task has seen the change and
  debounced it. Skip the settle and `home()` samples the pre-nudge state and concludes
  every lane is already home — the exact stale-level failure §7.1 exists to prevent.
- **`game.rs`** — owns `Motors`, the FSM, and per-race randomization (`SmallRng` seeded
  from `RoscRng`). Publishes the current `Mode` via
  `Signal<CriticalSectionRawMutex, Mode>` for the renderer.
- **`led_task`** — owns `PioWs2812`; on a ~30 ms ticker reads `RACE_VIEW` and draws (§5.4).
- **`calibrate.rs`, `audio.rs`** — see §6, §9.
- Single-core executor is plenty. Feed the `Watchdog` (~8 s) at the top of the game loop.

### State machine

| State | Behavior | → next |
|-------|----------|--------|
| *(boot only)* | one-shot forward nudge off the home switches (§7.1.1) | Home |
| **Home** | seed from switch *levels*; if any lane is short, `reverse_all(homing)` until every home switch is closed, then cut power | Attract |
| **Attract** | slow marquee chase (§5.5); wait for any `Select` | Selecting |
| **Selecting** | track current pick; `Select` updates it; light its lane | on `Go` (pick set) → Race |
| **Race** | signal lights + music, hold the field `RACE_START_DELAY_MS`, then kick and run per-lane speed segments that re-roll mid-race (§7.3); await first `EndHit` (overall `with_timeout`) | on `EndHit(w)` → Winner(w); on timeout → Home |
| **Winner(w)** | `brake_all`; winner flash/chase; (future: sound); ~4 s | Home |
| **Tune** | calibration UX (§6); entered only via GO-held-at-boot | Home (on save) |

Input gating: ignore `Select/Go` during Race/Home; ignore `StartHit/EndHit` when not
racing/homing.

### 7.2 Race start — lights and music lead the ducks

Pressing GO does **not** launch the field immediately. In order:

1. `Mode::Race` is signalled and `Sound::Race` fires — the marquee (§5.5) starts at full
   speed and the music begins.
2. The field is **held for `RACE_START_DELAY_MS`** (default 1 s), watchdog fed, motors
   enabled but at zero.
3. The event queue is drained (*after* the hold, so anything that arrived during it goes
   too), then the launch kick fires and the speed segments below begin.

Two reasons for the hold: the DY-SV8F takes a moment to spin up after a play command, so
launching on the same instant means the ducks beat the music out of the gate; and the beat
of anticipation is simply better showmanship. The renderer is a separate task, so the
marquee animates normally throughout the hold.

The race clock (`RACE_TIMEOUT_MS`) starts **after** the hold, so the delay doesn't eat into
the timeout budget.

### 7.3 Race speed model — segments, not a single roll

Speed varies **during** the race, not once at the start. Each lane runs an independent
sequence of *segments*; when a lane's segment expires it re-rolls, and the lanes are
scheduled independently, so the lead changes hands. Nothing is pre-determined — the
finish switch alone decides the winner.

Per re-roll, a lane either:
- **runs** at `baseline ± SPEED_SPREAD_PCT %` of its own calibrated baseline, clamped to
  `FLOOR_PCT..=100`, for `SEGMENT_MIN_MS..=SEGMENT_MAX_MS`; or
- **stalls** (probability `STALL_CHANCE_PCT`) at duty 0 for
  `STALL_MIN_MS..=STALL_MAX_MS`.

Four details that matter more than they look:

1. **Stalls coast, they don't brake.** Duty 0 leaves both DRV8833 inputs low, so the duck
   drifts to a stop. Braking both inputs high would stop it dead and read as a fault.
2. **Leaving a stall gets a `KICK_PCT` kick for `RESUME_KICK_MS`.** `FLOOR_PCT` is the
   minimum *moving* duty; a lane that has actually stopped has to break static friction
   again, and at these floors it may not restart without it.
3. **No stall in the opening segment.** A duck sitting still off the line reads as a
   broken machine rather than as drama, so the first segment is always a running speed.
4. **Nothing renders duck position.** Varying speed mid-race broke the old `elapsed ×
   duty` LED dead-reckoning, which is only valid while duty is constant. Rather than
   integrate `duty × dt` per frame to keep a synthetic readout honest, the race lights
   became a marquee chase and the position tracking was removed outright (§5.4).

**Race-time cost of stalls** ≈ `STALL_CHANCE_PCT × (mean stall ÷ mean segment)`. At the
defaults that's ~7 % of the race spent stopped, so `RACE_TIMEOUT_MS` must stay comfortably
above the nominal duration (12 s against a ~6 s nominal — plenty).

**Tuning the feel:** shorter `SEGMENT_*` = twitchier, more lead changes; higher
`SPEED_SPREAD_PCT` = bigger gaps; higher `STALL_CHANCE_PCT` = more comedy, longer races.
`FLOOR_PCT` sets the floor of the *running* range — if it sits close to the baseline the
spread gets clipped and slow ducks bunch at exactly the floor, so keep
`baseline × (1 − spread) > FLOOR_PCT`.

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
pub const NOMINAL_SECS:     f32 = 5.0; →  (removed — no position readout to scale, §5.4)
pub const RESET_TIMEOUT_MS: u64 = 8_000; → 10_000
pub const JOG_DUTY_PCT:      u8 = 100; →  35      // ⚠ safety: no software end stop in test-motors
```

`PWM_TOP` (6249 ≈ 20 kHz), `RACE_TIMEOUT_MS`, `WINNER_SHOW_MS`, `DEBOUNCE_MS`, `FRAME_MS`
and `WATCHDOG_MS` are unaffected by the hardware changes.

---

## 9. Audio — **DY-SV8F** over UART0 (`audio.rs`) — implemented

Serial-controlled MP3 module with a built-in 5 W class-D amp, playing from **onboard
8 MB flash** (no microSD). Datasheet: `Reference/DY-SV8F.pdf`. Replaced the originally
planned DFPlayer Mini; the `AudioSink` trait and the game's trigger points are unchanged,
only the frame format differs.

### 9.1 Wiring & module configuration

| | |
|---|---|
| Serial | **9600 8N1**, Pico **GP0 (UART0 TX) → module RXD/IO1 (pin 4)**, common GND |
| Module TXD (pin 3) | → GP1 if status queries are ever wanted. **The driver is TX-only** — the game never needs a reply |
| Mode | **UART mode**, selected by the board's 3-way DIP (CON1/2/3) **at power-up**, not in software |
| Power | 5 V; decouple it — the amp pulls real current on transients and shares the rail with the motors |
| Speaker | 4 Ω, 3–5 W on the module's own amp; there is also a **hardware volume trimpot** on the board, independent of the software volume |
| Optional | `BUSY` (pin 11) → **GP15**, the spare, if the game should ever know when a clip ends |

> **Datasheet caveats worth knowing.** (1) Its Work Mode Configuration table prints the
> same `CON3 CON2 CON1 = 1 0 0` for *both* UART Mode and One-Line Mode — one of them is a
> typo, so confirm the mode empirically (if the module ignores UART frames, try the other
> DIP position). (2) The pin-definition table says `BUSY` is **LOW while playing**, while
> the I/O-mode tables say it's high — verify before relying on it.

### 9.2 Protocol

Frames are `AA <cmd> <len> <data…> <sum>`, where `sum` is the **low byte of the arithmetic
sum** of every preceding byte. (Not the DFPlayer's `7E…EF` / two's-complement scheme.)

| Purpose | Frame | Notes |
|---|---|---|
| Select drive | `AA 0B 01 02 B8` | `02` = onboard FLASH (USB=00, SD=01) |
| Play track *n* | `AA 07 02 <hi> <lo> <sum>` | "Specified Song" — `n` is the filename number |
| Set volume | `AA 13 01 <vol> <sum>` | `vol` 0..=30, module default 20 |
| Set play mode | `AA 18 01 02 C5` | `02` = single-stop (play once, then stop) |
| Stop | `AA 04 00 AE` | |
| Query status | `AA 01 00 AB` | replies `AA 01 01 <00 stop\|01 play\|02 pause> <sum>` — needs RX |

`init()` runs select-drive → set-mode → set-volume at startup, after a short delay (the
module isn't ready to accept commands the instant power comes up).

### 9.3 Track map — **name the files exactly like this**

The DY-SV8F selects a track by the **number in its 5-digit filename**, so numbering is
determined by what you call the files:

| File | `Sound` | Played when |
|------|---------|-------------|
| `00001.mp3` | `Bet(0)` | duck 0 selected |
| `00002.mp3` | `Bet(1)` | duck 1 selected |
| `00003.mp3` | `Bet(2)` | duck 2 selected |
| `00004.mp3` | `Bet(3)` | duck 3 selected |
| `00005.mp3` | `Race`  | race starts |
| `00006.mp3` | `Win`   | **the player's duck won** |
| `00007.mp3` | `Lose`  | a different duck won |

- `Sound::Attract` and `Sound::Home` map to **no track** (silent). The call sites exist, so
  assigning clips later is one line in `Sound::track()`.
- **Copy the files onto the flash in numeric order anyway.** Filename numbering is what the
  datasheet documents, but some DY-family firmware indexes by FAT write order — copying in
  order costs nothing and makes both behaviours agree.
- Win vs. lose is relative to **the duck the player picked**, so `show_winner` takes the
  pick as well as the winner. A race that times out with no finisher plays nothing.

### 9.4 Interface

```rust
pub enum Sound { Bet(u8), Race, Win, Lose, Attract, Home }
pub trait AudioSink { fn play(&mut self, s: Sound); fn set_volume(&mut self, v: u8); }
pub struct DySv8f<'d> { /* UartTx<'d, Blocking>, volume */ }
pub struct NullAudio;  // logs via defmt; swap in if the module is unplugged
```

`game::run` is generic over `A: AudioSink`, so swapping backends is a one-word change in
`main.rs`. Writes use `blocking_write` into the UART's 32-byte TX FIFO — a 6-byte frame
never fills it, so nothing stalls the executor or the LED renderer (§2.1).

### 9.5 Bring-up — `--features test-audio`

Needs only the module, the button panel and a speaker; no motors, no limit switches.

| Control | Action |
|---|---|
| Duck button *N* | play that duck's bet clip directly (track *N*+1) |
| **GO** | play the **next** clip in the whole set, cycling `00001`…`00007` |
| **TUNE up / down** | volume ± 2 (0..=30) |

Every trigger logs the track number it requested, so comparing what you hear against §9.3
catches an off-by-one in the file numbering immediately. If *nothing* plays: check the DIP
straps are on UART mode, GP0 really lands on the module's **RXD**, the grounds are common,
and the speaker is on the module's own amp output.

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
14. **Boot safety** — ✅ *resolved.* Home slowly on power-up; a lane already resting on
    its home switch is detected from the switch **level**, not an arrival edge, and skips
    the move entirely (§7.1). A duck resting on a *finish* switch can't produce an instant
    win either: `race()` drains the event queue before it starts.

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
21. **DY-SV8F track numbering** follows the **5-digit filename** (`00003.mp3` = track 3),
    per the datasheet. Copy the clips onto the flash in numeric order anyway — some
    DY-family firmware indexes by write order instead, and copying in order makes both
    behaviours agree. Verify with `--features test-audio`. (§9.3, §9.5)
22. **DY-SV8F mode strapping** is latched at power-up from the CON1/2/3 DIP switch, not
    set in software. The datasheet prints the *same* code for UART and One-Line mode, so
    if the module ignores UART frames, try the other DIP position. (§9.1)
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
5. **`--features test-audio`** — DY-SV8F clip check; needs only the module, the button
   panel and a speaker, so it can be run at any point. Duck buttons play each bet clip,
   GO steps through all seven, TUNE up/down set volume. Confirms the UART straps, the
   wiring, and that the file numbering matches the map in §9.3. The full game uses the
   `DySv8f` backend by default — swap in `NullAudio` in `main.rs` to run silently.

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
