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
//!   test-audio  : DY-SV8F clip check — no motors or switches needed, just the module,
//!                 the button panel and a speaker.
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

    // Both legs poll switch LEVELS rather than waiting on arrival edges, so starting a
    // leg already parked on the switch it's driving toward reports instantly instead of
    // burning the full timeout.
    if crate::inputs::end_closed(lane) {
        defmt::info!("lane {} already at FINISH — skipping forward leg", lane);
    } else {
        defmt::info!("lane {} -> forward", lane);
        motors.enable(true);
        let mut d = [0u8; LANES];
        d[lane] = JOG_DUTY_PCT;
        motors.race_forward(d);
        let start = Instant::now();
        while !crate::inputs::end_closed(lane) {
            wdt.feed();
            let _ = with_timeout(Duration::from_millis(50), EVENTS.receive()).await;
            if start.elapsed() > Duration::from_millis(RACE_TIMEOUT_MS) {
                defmt::warn!("lane {} finish timeout — check end switch / motor", lane);
                break;
            }
        }
        defmt::info!("lane {} forward leg took {} ms", lane, start.elapsed().as_millis());
    }

    if crate::inputs::home_closed(lane) {
        defmt::info!("lane {} already HOME — skipping return leg", lane);
    } else {
        defmt::info!("lane {} -> home (reverse)", lane);
        motors.reverse_all(HOMING_PCT);
        let start = Instant::now();
        while !crate::inputs::home_closed(lane) {
            wdt.feed();
            let _ = with_timeout(Duration::from_millis(50), EVENTS.receive()).await;
            if start.elapsed() > Duration::from_millis(RESET_TIMEOUT_MS) {
                defmt::warn!("lane {} home timeout — check start switch / bumper", lane);
                break;
            }
        }
        defmt::info!("lane {} home leg took {} ms", lane, start.elapsed().as_millis());
    }
    motors.coast_all();
}

/// test-audio: verify the DY-SV8F wiring, mode straps, clip set and track numbering.
/// Needs no motors and no limit switches — just the module, the button panel, a speaker.
///
///   duck button N : play that duck's bet clip directly (track N+1)
///   GO            : play the NEXT clip in the full set, cycling 1..=7
///   UP / DOWN     : volume + / −
///
/// Every trigger logs the track number it asked for, so you can match what you hear
/// against the map in `audio.rs` and catch an off-by-one in the file numbering.
#[cfg(feature = "test-audio")]
pub async fn audio_check<A: crate::audio::AudioSink>(
    mut audio: A,
    selects: [Input<'static>; LANES],
    go: Input<'static>,
    up: Input<'static>,
    down: Input<'static>,
    mut wdt: Watchdog,
) -> ! {
    use crate::audio::{Sound, VOLUME_MAX};

    const CYCLE: [(Sound, &str); 7] = [
        (Sound::Bet(0), "00001.mp3  bet / duck 0"),
        (Sound::Bet(1), "00002.mp3  bet / duck 1"),
        (Sound::Bet(2), "00003.mp3  bet / duck 2"),
        (Sound::Bet(3), "00004.mp3  bet / duck 3"),
        (Sound::Race, "00005.mp3  race"),
        (Sound::Win, "00006.mp3  finish / WIN"),
        (Sound::Lose, "00007.mp3  finish / LOSE"),
    ];

    defmt::info!("BRINGUP audio (DY-SV8F). Duck button = that bet clip; GO = next clip; UP/DOWN = volume.");
    defmt::info!("If nothing plays at all: check the DIP straps are UART mode, GP0 -> module RXD, common GND, speaker on the module.");

    let mut idx = 0usize;
    let mut volume = crate::audio::VOLUME_DEFAULT;
    let mut last_sel = [false; LANES];
    let mut last_go = false;
    let mut last_up = false;
    let mut last_down = false;
    let mut ticks: u32 = 0;

    loop {
        wdt.feed();

        for (i, s) in selects.iter().enumerate() {
            let now = s.is_low();
            if now && !last_sel[i] {
                defmt::info!("duck {} -> {}", i, CYCLE[i].1);
                audio.play(Sound::Bet(i as u8));
            }
            last_sel[i] = now;
        }

        let g = go.is_low();
        if g && !last_go {
            let (sound, label) = CYCLE[idx];
            defmt::info!("GO -> [{}/{}] {}", idx + 1, CYCLE.len(), label);
            audio.play(sound);
            idx = (idx + 1) % CYCLE.len();
        }
        last_go = g;

        let u = up.is_low();
        if u && !last_up {
            volume = (volume + 2).min(VOLUME_MAX);
            defmt::info!("volume -> {}/{}", volume, VOLUME_MAX);
            audio.set_volume(volume);
        }
        last_up = u;

        let d = down.is_low();
        if d && !last_down {
            volume = volume.saturating_sub(2);
            defmt::info!("volume -> {}/{}", volume, VOLUME_MAX);
            audio.set_volume(volume);
        }
        last_down = d;

        ticks = ticks.wrapping_add(1);
        if ticks % 250 == 0 {
            defmt::info!("audio check alive: next GO plays [{}/{}]", idx + 1, CYCLE.len());
        }
        Timer::after(Duration::from_millis(20)).await;
    }
}
