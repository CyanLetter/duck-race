//! Central tuning constants and the LED layout — the one place to change the feel.
//! Pin assignments live in `main.rs` (they're peripheral fields on `Peripherals`);
//! this module holds the numeric knobs. See IMPLEMENTATION.md §8.

/// Number of lanes / ducks.
pub const LANES: usize = 4;

// ---- Motor PWM ------------------------------------------------------------
// Tuned for **N20 6 V / 500 RPM** gearmotors on a 5 V rail with a 20 T GT2 pulley
// (40 mm of belt per motor revolution). At 5 V the motors turn ~415 RPM no-load, so
// 100 % duty is ~277 mm/s (~11 in/s) — the 4 in/s target lands near 36 % duty and the
// whole game lives in a ~25–50 % band. See IMPLEMENTATION.md §4.3.

/// PWM period. TOP=6249 at 125 MHz sys / divider 1 ≈ 20 kHz — above audible whine.
pub const PWM_TOP: u16 = 6249;
/// DC motors won't creep smoothly near 0 — never command a moving motor below this.
/// The N20's low breakaway lets this sit far below the TT-era 35 %. **Estimate**:
/// measure the real minimum-reliable-moving duty per lane in TUNE and set ~5 above it.
pub const FLOOR_PCT: u8 = 70;
/// Brief high duty at launch to break static friction, then settle to target.
/// Rule of thumb: ~1.5× the baseline, briefly. (A 90 % kick on an N20 is an 11 in/s
/// lurch that eats 5 % of the track in the first frame.)
pub const KICK_PCT: u8 = 85;
pub const KICK_MS: u64 = 60;
/// Homing duty — the light V-wheel gantry needs little torque, and the N20's low stall
/// torque keeps the gentle stall-into-home-bumper soft (shared-reverse scheme, §4).
/// 40 % ≈ 111 mm/s, so a full-length lane homes in ~5.5 s.
pub const HOMING_PCT: u8 = 80;
/// Default per-lane baseline (overwritten by flash-persisted calibration).
/// 36 % ≈ 102 mm/s ≈ 4 in/s → ~6 s over the 610 mm track.
pub const BASE_DEFAULT_PCT: u8 = 80;
/// Per-race random speed spread, as a percentage **of the lane's baseline** (± this %).
/// Relative rather than absolute duty points so it survives motor swaps and per-lane
/// calibration untouched — 20 % of 36 ≈ ±7 points, the same feel as the old ±12 on 60.
pub const SPEED_SPREAD_PCT: u8 = 20;

// ---- Timing ---------------------------------------------------------------
/// Nominal race duration at the default baseline — used only to scale the LED
/// progress animation (the real winner is decided by the finish switch).
pub const NOMINAL_SECS: f32 = 6.0;
pub const RACE_TIMEOUT_MS: u64 = 12_000;
/// Worst case is homing from the finish line at HOMING_PCT (~5.5 s), plus margin.
pub const RESET_TIMEOUT_MS: u64 = 10_000;
pub const WINNER_SHOW_MS: u64 = 4_000;
pub const DEBOUNCE_MS: u64 = 25;
pub const FRAME_MS: u64 = 30; // ~33 fps LED tick
pub const WATCHDOG_MS: u64 = 8_000;

/// Convert a duty percentage to a PWM compare value.
pub const fn pct_to_compare(pct: u8) -> u16 {
    (PWM_TOP as u32 * pct as u32 / 100) as u16
}

/// Duty used by the bring-up motor-jog and single-lane test modes. Edit to taste.
/// **Keep this low.** `test-motors` has no software end stop, and at 100 % an N20 lane
/// traverses the full 610 mm in ~2.2 s — the mechanical bumpers are the only backstop.
#[cfg(any(feature = "test-motors", feature = "test-lane"))]
pub const JOG_DUTY_PCT: u8 = 80;

// ---- LED layout (4 serpentine rows, one per lane) -------------------------
// Side-on cabinet: lanes are stacked vertically and each gets its OWN row — no shared
// rows, `lane == row` 1:1. All rows are the same physical length, but keep them
// per-row so trimming differences don't break the mapping and so NUM_LEDS is a single
// source of truth.
// **Measure the real strips after the build and edit these, then rebuild** — the
// count is a compile-time const generic on the WS2812 driver. See IMPLEMENTATION.md §5.
pub const ROWS: usize = LANES;
pub const COUNTS: [usize; ROWS] = [18, 18, 18, 18];
pub const NUM_LEDS: usize = COUNTS[0] + COUNTS[1] + COUNTS[2] + COUNTS[3];
