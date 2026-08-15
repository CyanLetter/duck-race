//! Game state machine: HOME → ATTRACT → SELECT → RACE → WINNER → (home). No payout.
//! Runs inline in `main` (never returns) so the PIO `Common` stays alive for `led_task`.
//! See IMPLEMENTATION.md §7.

use embassy_rp::clocks::RoscRng;
use embassy_rp::watchdog::Watchdog;
use embassy_time::{with_timeout, Duration, Instant, Timer};

use crate::audio::{AudioSink, NullAudio, Sound};
use crate::calibrate::{self, Baselines, CalFlash};
use crate::config::{
    FLOOR_PCT, HOMING_PCT, KICK_MS, KICK_PCT, LANES, NOMINAL_SECS, RACE_TIMEOUT_MS,
    RESET_TIMEOUT_MS, SPEED_SPREAD_PCT, WINNER_SHOW_MS, BASE_DEFAULT_PCT,
};
use crate::inputs::{recv, Event, EVENTS};
use crate::leds::{Mode, RaceView, RACE_VIEW};
use crate::motors::Motors;

/// Never-returning game driver.
pub async fn run(
    mut motors: Motors<'static>,
    mut wdt: Watchdog,
    mut flash: CalFlash,
    mut base: Baselines,
    boot_tune: bool,
) {
    let mut audio = NullAudio;

    if boot_tune {
        calibrate::tune_mode(&mut motors, &mut wdt, &mut flash, &mut base).await;
    }

    loop {
        home(&mut motors, &mut wdt, &mut audio).await;
        let _pick = select(&mut wdt, &mut audio).await;
        let winner = race(&mut motors, &mut wdt, &mut audio, &base).await;
        show_winner(&mut motors, &mut wdt, &mut audio, winner).await;
    }
}

/// Reverse all lanes into their home bumpers until every start switch trips (shared
/// reverse line → early arrivers gently stall against the bumper until all are home).
async fn home(motors: &mut Motors<'_>, wdt: &mut Watchdog, audio: &mut NullAudio) {
    RACE_VIEW.signal(RaceView { mode: Mode::Home, progress: [0.0; LANES] });
    audio.play(Sound::Home);
    motors.enable(true);
    motors.reverse_all(HOMING_PCT);

    let mut homed = [false; LANES];
    let mut remaining = LANES;
    let start = Instant::now();
    while remaining > 0 {
        wdt.feed();
        if let Ok(Event::StartHit(l)) =
            with_timeout(Duration::from_millis(500), EVENTS.receive()).await
        {
            let l = l as usize;
            if l < LANES && !homed[l] {
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
async fn select(wdt: &mut Watchdog, audio: &mut NullAudio) -> usize {
    RACE_VIEW.signal(RaceView { mode: Mode::Attract, progress: [0.0; LANES] });

    // Wait for the first selection.
    let mut pick = loop {
        if let Event::Select(d) = recv(wdt).await {
            break (d as usize).min(LANES - 1);
        }
    };
    RACE_VIEW.signal(RaceView { mode: Mode::Select(pick as u8), progress: [0.0; LANES] });
    audio.play(Sound::Bet);

    loop {
        match recv(wdt).await {
            Event::Select(d) => {
                pick = (d as usize).min(LANES - 1);
                RACE_VIEW.signal(RaceView { mode: Mode::Select(pick as u8), progress: [0.0; LANES] });
                audio.play(Sound::Bet);
            }
            Event::Go => return pick,
            _ => {}
        }
    }
}

/// Run the race. Returns the winning lane, or None on timeout.
async fn race(
    motors: &mut Motors<'_>,
    wdt: &mut Watchdog,
    audio: &mut NullAudio,
    base: &Baselines,
) -> Option<usize> {
    // Per-lane duty = baseline ± a random offset that is a PERCENTAGE OF THAT BASELINE,
    // not a fixed number of duty points — so the feel survives motor swaps and per-lane
    // calibration (the N20s moved the baseline from ~60 % to ~36 %). Clamped to the floor.
    let mut rng = RoscRng;
    let mut duties = [0u8; LANES];
    for l in 0..LANES {
        let spread = (base.pct[l] as i32 * SPEED_SPREAD_PCT as i32 / 100).max(1);
        let off = (rng.next_u32() % (2 * spread as u32 + 1)) as i32 - spread;
        duties[l] = (base.pct[l] as i32 + off).clamp(FLOOR_PCT as i32, 100) as u8;
    }
    defmt::info!("race duties {}", duties);

    audio.play(Sound::Race);
    motors.enable(true);
    motors.race_forward([KICK_PCT; LANES]); // launch kick
    Timer::after(Duration::from_millis(KICK_MS)).await;
    motors.race_forward(duties);

    let start = Instant::now();
    let mut progress = [0.0f32; LANES];
    let winner;
    loop {
        wdt.feed();
        // Visual progress ∝ elapsed × duty (real finish is decided by the switch).
        let el = start.elapsed().as_millis() as f32 / 1000.0;
        for l in 0..LANES {
            progress[l] =
                (el * duties[l] as f32 / (NOMINAL_SECS * BASE_DEFAULT_PCT as f32)).min(1.0);
        }
        RACE_VIEW.signal(RaceView { mode: Mode::Race, progress });

        match with_timeout(Duration::from_millis(crate::config::FRAME_MS), EVENTS.receive()).await
        {
            Ok(Event::EndHit(l)) => {
                winner = Some((l as usize).min(LANES - 1));
                break;
            }
            Ok(_) => {}
            Err(_) => {} // frame tick — recompute progress
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

async fn show_winner(
    motors: &mut Motors<'_>,
    wdt: &mut Watchdog,
    audio: &mut NullAudio,
    winner: Option<usize>,
) {
    match winner {
        Some(w) => {
            defmt::info!("winner: duck {}", w);
            audio.play(Sound::Finish);
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
