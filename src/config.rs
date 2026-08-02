//! Central tuning constants and the LED layout — the one place to change the feel.
//! Pin assignments live in `main.rs` (they're peripheral fields on `Peripherals`);
//! this module holds the numeric knobs. See IMPLEMENTATION.md §8.

/// Number of lanes / ducks.
pub const LANES: usize = 4;

// ---- Motor PWM ------------------------------------------------------------
/// PWM period. TOP=6249 at 125 MHz sys / divider 1 ≈ 20 kHz — above audible whine.
pub const PWM_TOP: u16 = 6249;
/// DC motors won't creep smoothly near 0 — never command a moving motor below this.
pub const FLOOR_PCT: u8 = 35;
/// Brief high duty at launch to break static friction, then settle to target.
pub const KICK_PCT: u8 = 90;
pub const KICK_MS: u64 = 120;
/// Homing duty — low, because the light V-wheel gantry needs little torque, which
/// keeps the gentle stall-into-home-bumper soft (shared-reverse scheme, §4).
pub const HOMING_PCT: u8 = 35;
/// Default per-lane baseline (overwritten by flash-persisted calibration).
pub const BASE_DEFAULT_PCT: u8 = 60;
/// Per-race random speed spread applied around each lane's baseline (± this %).
pub const SPEED_SPREAD_PCT: u8 = 12;

// ---- Timing ---------------------------------------------------------------
/// Nominal race duration at the default baseline — used only to scale the LED
/// progress animation (the real winner is decided by the finish switch).
pub const NOMINAL_SECS: f32 = 5.0;
pub const RACE_TIMEOUT_MS: u64 = 12_000;
pub const RESET_TIMEOUT_MS: u64 = 8_000;
pub const WINNER_SHOW_MS: u64 = 4_000;
pub const DEBOUNCE_MS: u64 = 25;
pub const FRAME_MS: u64 = 30; // ~33 fps LED tick
pub const WATCHDOG_MS: u64 = 8_000;

/// Convert a duty percentage to a PWM compare value.
pub const fn pct_to_compare(pct: u8) -> u16 {
    (PWM_TOP as u32 * pct as u32 / 100) as u16
}

// ---- LED layout (5 serpentine columns for 4 lanes) ------------------------
// All columns are the same physical length, but keep them per-column so trimming
// differences don't break the mapping and so NUM_LEDS is a single source of truth.
// **Measure the real strips after the build and edit these, then rebuild** — the
// count is a compile-time const generic on the WS2812 driver. See IMPLEMENTATION.md §5.
pub const COUNTS: [usize; 5] = [18, 18, 18, 18, 18];
pub const NUM_LEDS: usize =
    COUNTS[0] + COUNTS[1] + COUNTS[2] + COUNTS[3] + COUNTS[4];
