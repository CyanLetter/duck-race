//! Bring-up test modes, each behind a cargo feature so you can validate the build one
//! lane assembly at a time, in assembly order. See IMPLEMENTATION.md §11.
//!
//!   test-motors : motor on track, NO switches. Tap a duck button to pick the lane,
//!                 hold UP = forward, hold DOWN = reverse, at JOG_DUTY_PCT. (Reverse uses
//!                 the shared line → drives every *connected* motor; fine one-at-a-time.)
//!   test-lane   : add the lane's limit switches. Tap a duck button to pick the lane,
//!                 GO = drive to the finish switch then return to the home switch.
//!   test-leds   : (LedController::test_walk, in leds.rs) serpentine wiring check.
//!
//! Enable exactly one, e.g. `cargo run --release --features test-motors`.

use embassy_rp::gpio::Input;
use embassy_rp::watchdog::Watchdog;
use embassy_time::{with_timeout, Duration, Instant, Timer};

use crate::config::LANES;
#[cfg(any(feature = "test-motors", feature = "test-lane"))]
use crate::config::JOG_DUTY_PCT;
use crate::inputs::{recv, Event, EVENTS};
use crate::motors::Motors;

/// test-motors: hold-to-run jog with no limit switches required.
#[cfg(feature = "test-motors")]
pub async fn motor_jog(
    mut motors: Motors<'static>,
    selects: [Input<'static>; LANES],
    fwd: Input<'static>,
    rev: Input<'static>,
    mut wdt: Watchdog,
) -> ! {
    defmt::info!(
        "BRINGUP motor jog @ {}%. Tap duck button = pick lane. Hold UP=fwd, DOWN=rev.",
        JOG_DUTY_PCT
    );
    defmt::info!("No end stops yet — use short taps so the gantry can't run off the rail!");
    motors.enable(true);
    let mut active = 0usize;
    loop {
        wdt.feed();
        for (i, s) in selects.iter().enumerate() {
            if s.is_low() && active != i {
                active = i;
                defmt::info!("active lane = {}", active);
            }
        }
        if fwd.is_low() {
            let mut d = [0u8; LANES];
            d[active] = JOG_DUTY_PCT;
            motors.race_forward(d); // active lane forward, others 0
        } else if rev.is_low() {
            motors.reverse_all(JOG_DUTY_PCT); // shared reverse line
        } else {
            motors.coast_all();
        }
        Timer::after(Duration::from_millis(20)).await;
    }
}

/// test-lane: run one lane to its finish switch and back home. Needs the input tasks
/// (duck buttons, GO, and this lane's start/end switches) spawned in `main`.
#[cfg(feature = "test-lane")]
pub async fn lane_sequence(mut motors: Motors<'static>, mut wdt: Watchdog) -> ! {
    defmt::info!(
        "BRINGUP single-lane @ {}%. Tap duck button = pick lane, GO = to-finish-then-home.",
        JOG_DUTY_PCT
    );
    motors.enable(true);
    let mut active = 0usize;
    loop {
        match recv(&mut wdt).await {
            Event::Select(d) => {
                active = (d as usize).min(LANES - 1);
                defmt::info!("active lane = {}", active);
            }
            Event::Go => run_to_end_and_home(&mut motors, &mut wdt, active).await,
            _ => {}
        }
    }
}

#[cfg(feature = "test-lane")]
async fn run_to_end_and_home(motors: &mut Motors<'_>, wdt: &mut Watchdog, lane: usize) {
    use crate::config::{HOMING_PCT, RACE_TIMEOUT_MS, RESET_TIMEOUT_MS};

    defmt::info!("lane {} -> forward", lane);
    motors.enable(true);
    let mut d = [0u8; LANES];
    d[lane] = JOG_DUTY_PCT;
    motors.race_forward(d);
    let start = Instant::now();
    loop {
        wdt.feed();
        if let Ok(Event::EndHit(l)) =
            with_timeout(Duration::from_millis(200), EVENTS.receive()).await
        {
            if l as usize == lane {
                defmt::info!("lane {} reached FINISH in {} ms", lane, start.elapsed().as_millis());
                break;
            }
        }
        if start.elapsed() > Duration::from_millis(RACE_TIMEOUT_MS) {
            defmt::warn!("lane {} finish timeout — check end switch / motor", lane);
            break;
        }
    }

    defmt::info!("lane {} -> home (reverse)", lane);
    motors.reverse_all(HOMING_PCT);
    let start = Instant::now();
    loop {
        wdt.feed();
        if let Ok(Event::StartHit(l)) =
            with_timeout(Duration::from_millis(200), EVENTS.receive()).await
        {
            if l as usize == lane {
                defmt::info!("lane {} HOME in {} ms", lane, start.elapsed().as_millis());
                break;
            }
        }
        if start.elapsed() > Duration::from_millis(RESET_TIMEOUT_MS) {
            defmt::warn!("lane {} home timeout — check start switch / bumper", lane);
            break;
        }
    }
    motors.coast_all();
}
