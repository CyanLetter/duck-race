//! Duck Race — RP2040 / Embassy firmware entry point.
//!
//! Top-down pinball-style duck racer: select a duck, GO, four ducks race at randomized
//! speeds, first to trip its finish switch wins, winner is shown, machine re-homes.
//! No payout — prizes are handed out by hand. See ../IMPLEMENTATION.md for the full plan.
//!
//! Normal build runs the game (tasks: `led_task` + one input task per button/switch;
//! game state machine runs inline in `main` so it never returns and the PIO `Common`
//! backing the LED driver stays alive). The `test-*` features instead run a single
//! bring-up mode (src/bringup.rs) for assembling/validating one piece at a time.

#![no_std]
#![no_main]
// Bring-up builds intentionally leave the game/LED code paths unused.
#![cfg_attr(
    any(feature = "test-motors", feature = "test-lane", feature = "test-leds"),
    allow(dead_code, unused_imports, unused_variables, unused_mut)
)]

mod audio;
#[cfg(any(feature = "test-motors", feature = "test-lane"))]
mod bringup;
mod calibrate;
mod config;
mod game;
mod inputs;
mod leds;
mod motors;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Pull};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::pio_programs::ws2812::PioWs2812Program;
use embassy_rp::watchdog::Watchdog;
use embassy_time::{Duration, Timer};

use crate::config::WATCHDOG_MS;
use crate::inputs::{go_task, input_task, Event};
use crate::leds::{led_task, LedController};
use crate::motors::Motors;

use {defmt_rtt as _, panic_probe as _};

// Compile-time guard: only one bring-up mode at a time.
#[cfg(any(
    all(feature = "test-motors", feature = "test-lane"),
    all(feature = "test-motors", feature = "test-leds"),
    all(feature = "test-lane", feature = "test-leds"),
))]
compile_error!("enable only one of: test-motors, test-lane, test-leds");

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    defmt::info!("Duck Race booting");

    let mut watchdog = Watchdog::new(p.WATCHDOG);
    watchdog.start(Duration::from_millis(WATCHDOG_MS));

    // ===================== BRING-UP: motor jog (no switches) =====================
    #[cfg(feature = "test-motors")]
    {
        let motors = Motors::build(
            p.PWM_SLICE1, p.PIN_2, p.PIN_3, p.PWM_SLICE2, p.PIN_4, p.PIN_5, p.PWM_SLICE3,
            p.PIN_6, p.PIN_7,
        );
        let selects = [
            Input::new(p.PIN_10, Pull::Up),
            Input::new(p.PIN_11, Pull::Up),
            Input::new(p.PIN_12, Pull::Up),
            Input::new(p.PIN_13, Pull::Up),
        ];
        let fwd = Input::new(p.PIN_15, Pull::Up); // UP button = forward hold
        let rev = Input::new(p.PIN_16, Pull::Up); // DOWN button = reverse hold
        let fault = Input::new(p.PIN_8, Pull::Up); // DRV8833 nFAULT/ULT (low = fault)
        bringup::motor_jog(motors, selects, fwd, rev, fault, watchdog).await;
    }

    // ================= BRING-UP: single lane, to-finish-and-home =================
    #[cfg(feature = "test-lane")]
    {
        _spawner.must_spawn(input_task(Input::new(p.PIN_10, Pull::Up), Event::Select(0)));
        _spawner.must_spawn(input_task(Input::new(p.PIN_11, Pull::Up), Event::Select(1)));
        _spawner.must_spawn(input_task(Input::new(p.PIN_12, Pull::Up), Event::Select(2)));
        _spawner.must_spawn(input_task(Input::new(p.PIN_13, Pull::Up), Event::Select(3)));
        _spawner.must_spawn(go_task(Input::new(p.PIN_14, Pull::Up)));
        _spawner.must_spawn(input_task(Input::new(p.PIN_17, Pull::Up), Event::StartHit(0)));
        _spawner.must_spawn(input_task(Input::new(p.PIN_18, Pull::Up), Event::StartHit(1)));
        _spawner.must_spawn(input_task(Input::new(p.PIN_19, Pull::Up), Event::StartHit(2)));
        _spawner.must_spawn(input_task(Input::new(p.PIN_20, Pull::Up), Event::StartHit(3)));
        _spawner.must_spawn(input_task(Input::new(p.PIN_21, Pull::Up), Event::EndHit(0)));
        _spawner.must_spawn(input_task(Input::new(p.PIN_22, Pull::Up), Event::EndHit(1)));
        _spawner.must_spawn(input_task(Input::new(p.PIN_26, Pull::Up), Event::EndHit(2)));
        _spawner.must_spawn(input_task(Input::new(p.PIN_27, Pull::Up), Event::EndHit(3)));
        let motors = Motors::build(
            p.PWM_SLICE1, p.PIN_2, p.PIN_3, p.PWM_SLICE2, p.PIN_4, p.PIN_5, p.PWM_SLICE3,
            p.PIN_6, p.PIN_7,
        );
        bringup::lane_sequence(motors, watchdog).await;
    }

    // ===================== BRING-UP: LED serpentine walk =========================
    #[cfg(feature = "test-leds")]
    {
        let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);
        let program = PioWs2812Program::new(&mut common);
        let mut leds = LedController::new(&mut common, sm0, p.DMA_CH0, p.PIN_9, &program);
        leds.test_walk(&mut watchdog).await;
    }

    // =============================== NORMAL GAME =================================
    #[cfg(not(any(feature = "test-motors", feature = "test-lane", feature = "test-leds")))]
    {
        // Motors: 2× DRV8833, shared reverse line.
        let motors = Motors::build(
            p.PWM_SLICE1, p.PIN_2, p.PIN_3, p.PWM_SLICE2, p.PIN_4, p.PIN_5, p.PWM_SLICE3,
            p.PIN_6, p.PIN_7,
        );

        // LEDs: WS2812 chain on PIO0 SM0 / GP9 (via 74AHCT125).
        let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);
        let program = PioWs2812Program::new(&mut common);
        let leds = LedController::new(&mut common, sm0, p.DMA_CH0, p.PIN_9, &program);
        _spawner.must_spawn(led_task(leds));

        // Inputs. GO first: sample held-at-boot to enter TUNE, then hand to its task.
        let go = Input::new(p.PIN_14, Pull::Up);
        Timer::after(Duration::from_millis(50)).await;
        let boot_tune = go.is_low();
        _spawner.must_spawn(go_task(go));

        _spawner.must_spawn(input_task(Input::new(p.PIN_10, Pull::Up), Event::Select(0)));
        _spawner.must_spawn(input_task(Input::new(p.PIN_11, Pull::Up), Event::Select(1)));
        _spawner.must_spawn(input_task(Input::new(p.PIN_12, Pull::Up), Event::Select(2)));
        _spawner.must_spawn(input_task(Input::new(p.PIN_13, Pull::Up), Event::Select(3)));
        _spawner.must_spawn(input_task(Input::new(p.PIN_15, Pull::Up), Event::Up));
        _spawner.must_spawn(input_task(Input::new(p.PIN_16, Pull::Up), Event::Down));
        _spawner.must_spawn(input_task(Input::new(p.PIN_17, Pull::Up), Event::StartHit(0)));
        _spawner.must_spawn(input_task(Input::new(p.PIN_18, Pull::Up), Event::StartHit(1)));
        _spawner.must_spawn(input_task(Input::new(p.PIN_19, Pull::Up), Event::StartHit(2)));
        _spawner.must_spawn(input_task(Input::new(p.PIN_20, Pull::Up), Event::StartHit(3)));
        _spawner.must_spawn(input_task(Input::new(p.PIN_21, Pull::Up), Event::EndHit(0)));
        _spawner.must_spawn(input_task(Input::new(p.PIN_22, Pull::Up), Event::EndHit(1)));
        _spawner.must_spawn(input_task(Input::new(p.PIN_26, Pull::Up), Event::EndHit(2)));
        _spawner.must_spawn(input_task(Input::new(p.PIN_27, Pull::Up), Event::EndHit(3)));

        // Flash-persisted baselines.
        let mut flash = calibrate::new_flash(p.FLASH);
        let base = calibrate::load(&mut flash);
        if boot_tune {
            defmt::info!("GO held at boot → entering TUNE mode");
        }

        // Run the game inline (never returns; keeps PIO `common` alive).
        game::run(motors, watchdog, flash, base, boot_tune).await;
    }
}
