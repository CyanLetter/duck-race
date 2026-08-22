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
pub const HOMING_PCT: u8 = 65;
/// Default per-lane baseline (overwritten by flash-persisted calibration).
/// 36 % ≈ 102 mm/s ≈ 4 in/s → ~6 s over the 610 mm track.
pub const BASE_DEFAULT_PCT: u8 = 70;
/// Random speed spread, as a percentage **of the lane's baseline** (± this %).
/// Relative rather than absolute duty points so it survives motor swaps and per-lane
/// calibration untouched. Re-rolled on every race *segment*, not once per race.
pub const SPEED_SPREAD_PCT: u8 = 20;

// ---- In-race speed variation ----------------------------------------------
// Each lane runs a sequence of independent "segments". At the end of a segment it
// re-rolls: usually a fresh speed around its baseline, occasionally a brief stall. Lanes
// are scheduled independently, so leads change hands mid-race. Nothing is
// pre-determined — the finish switch decides the winner. See IMPLEMENTATION.md §7.2.

/// A running segment lasts a uniform-random time in this range.
pub const SEGMENT_MIN_MS: u64 = 200;
pub const SEGMENT_MAX_MS: u64 = 400;
/// Chance (percent) that a re-roll produces a stall instead of a new speed.
/// Cost in race time ≈ chance × stall/segment duration ratio — at 12 % that's ~7 % of
/// the race spent stopped, so keep `RACE_TIMEOUT_MS` comfortably above the nominal.
pub const STALL_CHANCE_PCT: u32 = 30;
/// A stall lasts a uniform-random time in this range. Stalls coast (both inputs low),
/// they don't brake — the duck drifts to a stop, which reads better than a hard stop.
pub const STALL_MIN_MS: u64 = 100;
pub const STALL_MAX_MS: u64 = 300;
/// After a stall the motor has to break static friction again, so a lane leaving a stall
/// gets `KICK_PCT` for this long before settling to its new speed.
pub const RESUME_KICK_MS: u64 = 90;

// ---- Boot ------------------------------------------------------------------
/// One-shot forward nudge at power-up, run once before the first homing pass.
///
/// A gantry can come to rest *just* shy of its home switch's trip point — close enough to
/// look parked, not close enough to close the contact — and reversing into an already
/// seated gantry won't reliably move it the last fraction of a millimetre. Backing every
/// lane off the home region first means the home run arrives with momentum and seats the
/// switch cleanly. Set `BOOT_NUDGE_MS = 0` to disable.
pub const BOOT_NUDGE_PCT: u8 = 85;
pub const BOOT_NUDGE_MS: u64 = 200;

// ---- Timing ---------------------------------------------------------------
/// Beat between GO and the ducks actually launching. Music and lights start immediately;
/// the field is held for this long. The audio module takes a moment to spin up, and the
/// anticipation reads better than ducks leaving before the music does.
pub const RACE_START_DELAY_MS: u64 = 1_000;
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
pub const COUNTS: [usize; ROWS] = [39, 39, 39, 39];
pub const NUM_LEDS: usize = COUNTS[0] + COUNTS[1] + COUNTS[2] + COUNTS[3];

// ---- Marquee chase (old carnival incandescent look) -----------------------
// Used for both RACE and ATTRACT — every row runs the same chase, in sync, so the whole
// board reads as one marquee. Bulbs snap on and fade slowly, which is the thing that
// separates "filament" from "LED". Race lights deliberately do NOT track duck position.

/// One bright bulb every N pixels along each row.
pub const CHASE_SPACING: usize = 5;
/// Time for the chase to advance by one pixel during a race. Lower = faster.
pub const CHASE_STEP_MS_RACE: u64 = 55;
/// Same chase, slower, while idle.
pub const CHASE_STEP_MS_ATTRACT: u64 = 150;
/// Exponential fade time constant, measured in pixel-steps. Larger = longer, lazier
/// tails. At 1.6 a bulb is down to ~54 % one step later and ~8 % four steps later, so the
/// tail has just faded out as the next bulb arrives.
pub const CHASE_TAU_STEPS: f32 = 1.6;
