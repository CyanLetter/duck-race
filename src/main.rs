//! Duck Race — RP2040 / Embassy firmware entry point.
//!
//! Top-down pinball-style duck racer: select a duck, GO, four ducks race at randomized
//! speeds, first to trip its finish switch wins, winner is shown, machine re-homes.
//! No payout — prizes are handed out by hand. See ../IMPLEMENTATION.md for the full plan.
//!
//! Tasks: `led_task` (decoupled renderer) + one input task per button/switch. The game
//! state machine runs inline in `main` (so it never returns and the PIO `Common` that
//! backs the LED driver stays alive).

#![no_std]
#![no_main]

mod audio;
mod calibrate;
mod config;
mod game;
mod inputs;
mod leds;
mod motors;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::pio_programs::ws2812::PioWs2812Program;
use embassy_rp::pwm::{Config as PwmConfig, Pwm};
use embassy_rp::watchdog::Watchdog;
use embassy_time::{Duration, Timer};

use crate::config::{PWM_TOP, WATCHDOG_MS};
use crate::inputs::{go_task, input_task, Event};
use crate::leds::{led_task, LedController};
use crate::motors::Motors;

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    defmt::info!("Duck Race booting");

    // ---- Watchdog (fed by the game loop via inputs::recv) --------------------
    let mut watchdog = Watchdog::new(p.WATCHDOG);
    watchdog.start(Duration::from_millis(WATCHDOG_MS));

    // ---- Motors: 2× DRV8833, shared reverse line -----------------------------
    // GP2/3 = slice1 A/B (lanes 0/1), GP4/5 = slice2 A/B (lanes 2/3), GP6 = slice3 A rev.
    let mut pcfg = PwmConfig::default();
    pcfg.top = PWM_TOP;
    pcfg.divider = 1u8.into(); // ~20 kHz
    let fwd01 = Pwm::new_output_ab(p.PWM_SLICE1, p.PIN_2, p.PIN_3, pcfg.clone());
    let fwd23 = Pwm::new_output_ab(p.PWM_SLICE2, p.PIN_4, p.PIN_5, pcfg.clone());
    let rev = Pwm::new_output_a(p.PWM_SLICE3, p.PIN_6, pcfg.clone());
    let nsleep = Output::new(p.PIN_7, Level::Low); // DRV8833 nSLEEP (both boards)
    let motors = Motors::new(fwd01, fwd23, rev, pcfg, nsleep);

    // ---- LEDs: WS2812 chain on PIO0 SM0 / GP9 (via 74AHCT125) ----------------
    let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);
    let program = PioWs2812Program::new(&mut common);
    let leds = LedController::new(&mut common, sm0, p.DMA_CH0, p.PIN_9, &program);
    spawner.must_spawn(led_task(leds));

    // ---- Inputs --------------------------------------------------------------
    // GO first: sample it held-at-boot to enter TUNE mode, then hand to its task.
    let go = Input::new(p.PIN_14, Pull::Up);
    Timer::after(Duration::from_millis(50)).await;
    let boot_tune = go.is_low();
    spawner.must_spawn(go_task(go));

    // Duck-select buttons.
    spawner.must_spawn(input_task(Input::new(p.PIN_10, Pull::Up), Event::Select(0)));
    spawner.must_spawn(input_task(Input::new(p.PIN_11, Pull::Up), Event::Select(1)));
    spawner.must_spawn(input_task(Input::new(p.PIN_12, Pull::Up), Event::Select(2)));
    spawner.must_spawn(input_task(Input::new(p.PIN_13, Pull::Up), Event::Select(3)));
    // TUNE up/down.
    spawner.must_spawn(input_task(Input::new(p.PIN_15, Pull::Up), Event::Up));
    spawner.must_spawn(input_task(Input::new(p.PIN_16, Pull::Up), Event::Down));
    // Start (home) limit switches.
    spawner.must_spawn(input_task(Input::new(p.PIN_17, Pull::Up), Event::StartHit(0)));
    spawner.must_spawn(input_task(Input::new(p.PIN_18, Pull::Up), Event::StartHit(1)));
    spawner.must_spawn(input_task(Input::new(p.PIN_19, Pull::Up), Event::StartHit(2)));
    spawner.must_spawn(input_task(Input::new(p.PIN_20, Pull::Up), Event::StartHit(3)));
    // End (finish) limit switches.
    spawner.must_spawn(input_task(Input::new(p.PIN_21, Pull::Up), Event::EndHit(0)));
    spawner.must_spawn(input_task(Input::new(p.PIN_22, Pull::Up), Event::EndHit(1)));
    spawner.must_spawn(input_task(Input::new(p.PIN_26, Pull::Up), Event::EndHit(2)));
    spawner.must_spawn(input_task(Input::new(p.PIN_27, Pull::Up), Event::EndHit(3)));

    // ---- Flash-persisted baselines ------------------------------------------
    let mut flash = calibrate::new_flash(p.FLASH);
    let base = calibrate::load(&mut flash);
    if boot_tune {
        defmt::info!("GO held at boot → entering TUNE mode");
    }

    // ---- Run the game inline (never returns; keeps PIO `common` alive) -------
    game::run(motors, watchdog, flash, base, boot_tune).await;
}
