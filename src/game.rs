//! Game state machine: HOME → ATTRACT → SELECT → RACE → WINNER → (home). No payout.
//! Runs inline in `main` (never returns) so the PIO `Common` stays alive for `led_task`.
//! See IMPLEMENTATION.md §7.

use embassy_rp::clocks::RoscRng;
use embassy_rp::watchdog::Watchdog;
use embassy_time::{with_timeout, Duration, Instant, Timer};

use crate::audio::{AudioSink, Sound};
use crate::calibrate::{self, Baselines, CalFlash};
use crate::config::{
    BASE_DEFAULT_PCT, FLOOR_PCT, HOMING_PCT, KICK_MS, KICK_PCT, LANES, NOMINAL_SECS,
    RACE_TIMEOUT_MS, RESET_TIMEOUT_MS, RESUME_KICK_MS, SEGMENT_MAX_MS, SEGMENT_MIN_MS,
    SPEED_SPREAD_PCT, STALL_CHANCE_PCT, STALL_MAX_MS, STALL_MIN_MS, WINNER_SHOW_MS,
};
use crate::inputs::{self, recv, Event, EVENTS};
use crate::leds::{Mode, RaceView, RACE_VIEW};
use crate::motors::Motors;

/// Never-returning game driver.
pub async fn run<A: AudioSink>(
    mut motors: Motors<'static>,
    mut wdt: Watchdog,
    mut flash: CalFlash,
    mut base: Baselines,
    boot_tune: bool,
    mut audio: A,
) {
    if boot_tune {
        calibrate::tune_mode(&mut motors, &mut wdt, &mut flash, &mut base).await;
    }

    loop {
        home(&mut motors, &mut wdt, &mut audio).await;
        let pick = select(&mut wdt, &mut audio).await;
        let winner = race(&mut motors, &mut wdt, &mut audio, &base).await;
        show_winner(&mut motors, &mut wdt, &mut audio, pick, winner).await;
    }
}

/// Reverse all lanes into their home bumpers until every start switch is closed (shared
/// reverse line → early arrivers gently stall against the bumper until all are home).
///
/// Works off switch *levels*, not arrival events: a lane that is already resting on its
/// home switch (very common at power-up) never produces a falling edge, so an
/// events-only wait would hang until `RESET_TIMEOUT_MS` every single boot.
async fn home<A: AudioSink>(motors: &mut Motors<'_>, wdt: &mut Watchdog, audio: &mut A) {
    RACE_VIEW.signal(RaceView { mode: Mode::Home, progress: [0.0; LANES] });
    audio.play(Sound::Home);
    motors.enable(true);

    let mut homed = [false; LANES];
    let mut remaining = 0usize;
    for l in 0..LANES {
        homed[l] = inputs::home_closed(l);
        if !homed[l] {
            remaining += 1;
        }
    }
    if remaining == 0 {
        defmt::info!("all lanes already home — no movement needed");
        motors.coast_all();
        return;
    }
    defmt::info!("homing: {} lane(s) to go, already home = {}", remaining, homed);
    motors.reverse_all(HOMING_PCT);

    let start = Instant::now();
    while remaining > 0 {
        wdt.feed();
        // The timeout sets the poll cadence; receiving also keeps the event channel from
        // backing up while we're driving (input tasks block on a full channel).
        let _ = with_timeout(Duration::from_millis(50), EVENTS.receive()).await;
        for l in 0..LANES {
            if !homed[l] && inputs::home_closed(l) {
                homed[l] = true;
                remaining -= 1;
            }
        }
        if start.elapsed() > Duration::from_millis(RESET_TIMEOUT_MS) {
            defmt::warn!("home timeout; homed = {}", homed);
            break;
        }
    }
    motors.coast_all();
}

/// Attract until a duck is selected, then let the player change the pick until GO.
async fn select<A: AudioSink>(wdt: &mut Watchdog, audio: &mut A) -> usize {
    RACE_VIEW.signal(RaceView { mode: Mode::Attract, progress: [0.0; LANES] });

    // Wait for the first selection.
    let mut pick = loop {
        if let Event::Select(d) = recv(wdt).await {
            break (d as usize).min(LANES - 1);
        }
    };
    RACE_VIEW.signal(RaceView { mode: Mode::Select(pick as u8), progress: [0.0; LANES] });
    audio.play(Sound::Bet(pick as u8)); // one clip per duck

    loop {
        match recv(wdt).await {
            Event::Select(d) => {
                pick = (d as usize).min(LANES - 1);
                RACE_VIEW.signal(RaceView { mode: Mode::Select(pick as u8), progress: [0.0; LANES] });
                audio.play(Sound::Bet(pick as u8));
            }
            Event::Go => return pick,
            _ => {}
        }
    }
}

/// Uniform random integer in `lo..=hi`.
fn rand_range(rng: &mut RoscRng, lo: u64, hi: u64) -> u64 {
    if hi <= lo {
        return lo;
    }
    lo + (rng.next_u32() as u64) % (hi - lo + 1)
}

/// Roll a running speed for one lane: its calibrated baseline ± a random offset that is
/// a PERCENTAGE OF THAT BASELINE, not a fixed number of duty points — so the feel
/// survives motor swaps and per-lane calibration. Clamped to the floor.
fn roll_speed(rng: &mut RoscRng, base_pct: u8) -> u8 {
    let spread = (base_pct as i32 * SPEED_SPREAD_PCT as i32 / 100).max(1);
    let off = (rng.next_u32() % (2 * spread as u32 + 1)) as i32 - spread;
    (base_pct as i32 + off).clamp(FLOOR_PCT as i32, 100) as u8
}

/// Run the race. Returns the winning lane, or None on timeout.
///
/// Speed varies *during* the race: each lane runs independently-scheduled segments,
/// re-rolling a new speed (or occasionally a brief stall) at random intervals, so the
/// lead changes hands. Fully random — nothing is pre-determined and the finish switch
/// alone decides the winner. See IMPLEMENTATION.md §7.2.
async fn race<A: AudioSink>(
    motors: &mut Motors<'_>,
    wdt: &mut Watchdog,
    audio: &mut A,
    base: &Baselines,
) -> Option<usize> {
    let mut rng = RoscRng;

    // Winner detection stays EDGE-driven (unlike homing): the finish switches are the
    // one measurement where timing resolution matters, and an edge lands sooner and more
    // precisely than a polled level. That makes a stale queued edge dangerous, so start
    // from an empty channel — a leftover EndHit would otherwise win instantly.
    inputs::drain();

    audio.play(Sound::Race);
    motors.enable(true);
    motors.race_forward([KICK_PCT; LANES]); // launch kick, all lanes together
    Timer::after(Duration::from_millis(KICK_MS)).await;

    // Opening segment: every lane gets a real speed — no stall on the first leg, since a
    // duck sitting still off the line reads as a fault rather than as drama.
    let start = Instant::now();
    let mut target = [0u8; LANES]; // the speed this lane is currently trying to hold
    let mut next_change = [start; LANES]; // when this lane re-rolls
    let mut kick_until = [start; LANES]; // stall-recovery kick window
    for l in 0..LANES {
        target[l] = roll_speed(&mut rng, base.pct[l]);
        next_change[l] =
            start + Duration::from_millis(rand_range(&mut rng, SEGMENT_MIN_MS, SEGMENT_MAX_MS));
    }
    defmt::info!("race opening duties {}", target);

    let mut commanded = [0u8; LANES];
    let mut progress = [0.0f32; LANES];
    let mut last = start;
    let winner;
    loop {
        wdt.feed();
        let now = Instant::now();

        // --- re-roll any lane whose segment has expired ---
        for l in 0..LANES {
            if now < next_change[l] {
                continue;
            }
            let stalling = target[l] == 0;
            if !stalling && rng.next_u32() % 100 < STALL_CHANCE_PCT {
                // Enter a stall: coast (duty 0), don't brake — the duck drifts to a stop.
                let ms = rand_range(&mut rng, STALL_MIN_MS, STALL_MAX_MS);
                target[l] = 0;
                next_change[l] = now + Duration::from_millis(ms);
                defmt::info!("lane {} stalls for {} ms", l, ms);
            } else {
                // New running speed. Coming out of a stall, the motor has to break static
                // friction again, so open with a brief kick.
                if stalling {
                    kick_until[l] = now + Duration::from_millis(RESUME_KICK_MS);
                }
                target[l] = roll_speed(&mut rng, base.pct[l]);
                next_change[l] =
                    now + Duration::from_millis(rand_range(&mut rng, SEGMENT_MIN_MS, SEGMENT_MAX_MS));
            }
        }

        // --- what we actually drive this frame (stall-recovery kick overrides target) ---
        let cmd: [u8; LANES] = core::array::from_fn(|l| {
            if now < kick_until[l] { KICK_PCT.max(target[l]) } else { target[l] }
        });
        if cmd != commanded {
            motors.race_forward(cmd);
            commanded = cmd;
        }

        // --- visual progress: INTEGRATE the duty actually commanded, since it now
        //     changes mid-race (real finish is still decided by the switch) ---
        let dt = (now - last).as_micros() as f32 / 1_000_000.0;
        last = now;
        for l in 0..LANES {
            let step = cmd[l] as f32 * dt / (NOMINAL_SECS * BASE_DEFAULT_PCT as f32);
            progress[l] = (progress[l] + step).min(1.0);
        }
        RACE_VIEW.signal(RaceView { mode: Mode::Race, progress });

        match with_timeout(Duration::from_millis(crate::config::FRAME_MS), EVENTS.receive()).await
        {
            Ok(Event::EndHit(l)) => {
                winner = Some((l as usize).min(LANES - 1));
                break;
            }
            Ok(_) => {}
            Err(_) => {} // frame tick — re-roll, re-drive, recompute progress
        }
        if start.elapsed() > Duration::from_millis(RACE_TIMEOUT_MS) {
            winner = None;
            break;
        }
    }

    motors.brake_all();
    Timer::after(Duration::from_millis(200)).await;
    motors.coast_all();
    winner
}

async fn show_winner<A: AudioSink>(
    motors: &mut Motors<'_>,
    wdt: &mut Watchdog,
    audio: &mut A,
    pick: usize,
    winner: Option<usize>,
) {
    match winner {
        Some(w) => {
            // Win/lose is relative to the duck the player picked, not to lane 0.
            let won = w == pick;
            defmt::info!("winner: duck {} (player picked {}) -> {}", w, pick, if won { "WIN" } else { "LOSE" });
            audio.play(if won { Sound::Win } else { Sound::Lose });
            RACE_VIEW.signal(RaceView { mode: Mode::Winner(w as u8), progress: [1.0; LANES] });
        }
        None => defmt::warn!("race timed out with no finisher"),
    }
    let start = Instant::now();
    while start.elapsed() < Duration::from_millis(WINNER_SHOW_MS) {
        wdt.feed();
        Timer::after(Duration::from_millis(200)).await;
    }
    let _ = motors; // motors already coasting; kept for symmetry / future use
}
