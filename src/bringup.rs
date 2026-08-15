//! Bring-up test modes, each behind a cargo feature so you can validate the build one
//! lane assembly at a time, in assembly order. See IMPLEMENTATION.md §11.
//!
//!   test-motors : motor on track, NO switches. Tap a duck button to pick the lane,
//!                 hold UP = forward, hold DOWN = reverse, at JOG_DUTY_PCT. (Reverse uses
//!                 the shared line → drives every *connected* motor; fine one-at-a-time.)
//!                 Keep JOG_DUTY_PCT low — an N20 lane traverses 610 mm in ~2.2 s at
//!                 100 %, and there is no software end stop in this mode.
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
/// `fault` is the DRV8833 nFAULT/ULT line (open-drain, LOW = fault) on GP28, read with a
/// pull-up so we can tell if the driver has tripped.
#[cfg(feature = "test-motors")]
pub async fn motor_jog(
    mut motors: Motors<'static>,
    selects: [Input<'static>; LANES],
    fwd: Input<'static>,
    rev: Input<'static>,
    fault: Input<'static>,
    mut wdt: Watchdog,
) -> ! {
    defmt::info!(
        "BRINGUP motor jog @ {}%. Tap duck = pick lane. Hold UP(GP16)=fwd, DOWN(GP17)=rev.",
        JOG_DUTY_PCT
    );
    defmt::info!("No end stops yet — use short taps so the gantry can't run off the rail!");
    motors.enable(true);
    defmt::info!(
        "nSLEEP/EEP set HIGH (GP27=enable). nFAULT/ULT (GP28) now reads: {}",
        if fault.is_low() { "LOW = FAULT!" } else { "high = ok" }
    );

    let mut active = 0usize;
    let mut last_fwd = false;
    let mut last_rev = false;
    let mut last_fault = fault.is_low();
    let mut ticks: u32 = 0;

    loop {
        wdt.feed();

        // --- select (edge-logged) ---
        for (i, s) in selects.iter().enumerate() {
            if s.is_low() && active != i {
                active = i;
                defmt::info!("SELECT: active lane = {}", active);
            }
        }

        // --- fault line (edge-logged) ---
        let faulted = fault.is_low();
        if faulted && !last_fault {
            defmt::warn!("nFAULT asserted LOW — driver fault (check VM, overcurrent, thermal, wiring)");
        } else if !faulted && last_fault {
            defmt::info!("nFAULT cleared (high)");
        }
        last_fault = faulted;

        // --- jog buttons (edge-logged) ---
        let f = fwd.is_low();
        let r = rev.is_low();
        if f != last_fwd {
            if f {
                defmt::info!("UP (GP16) pressed -> lane {} FORWARD @ {}% (IN1=PWM, GP26=low)", active, JOG_DUTY_PCT);
            } else {
                defmt::info!("UP (GP16) released -> coast");
            }
            last_fwd = f;
        }
        if r != last_rev {
            if r {
                defmt::info!("DOWN (GP17) pressed -> REVERSE @ {}% (IN1=low, GP26=PWM)", JOG_DUTY_PCT);
            } else {
                defmt::info!("DOWN (GP17) released -> coast");
            }
            last_rev = r;
        }

        // --- drive ---
        if f {
            let mut d = [0u8; LANES];
            d[active] = JOG_DUTY_PCT;
            motors.race_forward(d); // active lane forward, others 0
        } else if r {
            motors.reverse_all(JOG_DUTY_PCT); // shared reverse line
        } else {
            motors.coast_all();
        }

        // --- heartbeat every ~2 s so we know the loop is alive ---
        ticks = ticks.wrapping_add(1);
        if ticks % 100 == 0 {
            defmt::info!(
                "jog alive: lane {}, UP={}, DOWN={}, nFAULT={}",
                active,
                if f { "down" } else { "up" },
                if r { "down" } else { "up" },
                if faulted { "LOW" } else { "ok" }
            );
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
