//! Buttons and limit switches → debounced events on a shared channel.
//!
//! All inputs are active-low with internal pull-ups (press / switch-close = falling
//! edge). One `input_task` per pin; GO gets its own task so it can distinguish a short
//! press (start race) from a long press (save & exit TUNE). See IMPLEMENTATION.md §7.

use embassy_futures::select::{select, Either};
use embassy_rp::gpio::Input;
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};

use crate::config::DEBOUNCE_MS;

#[derive(Clone, Copy)]
pub enum Event {
    Select(u8),   // duck-select button 0..LANES
    Go,           // short GO press
    GoLong,       // long GO press (TUNE: save & exit)
    Up,           // TUNE: nudge active lane up
    Down,         // TUNE: nudge active lane down
    StartHit(u8), // lane's start/home switch tripped
    EndHit(u8),   // lane's finish switch tripped
}

pub static EVENTS: Channel<CriticalSectionRawMutex, Event, 16> = Channel::new();

const LONG_PRESS: Duration = Duration::from_millis(800);

/// Generic input: emit a fixed event on each debounced press, one event per press.
#[embassy_executor::task(pool_size = 16)]
pub async fn input_task(mut pin: Input<'static>, ev: Event) {
    loop {
        pin.wait_for_falling_edge().await;
        Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
        if pin.is_low() {
            EVENTS.send(ev).await;
            pin.wait_for_high().await; // require release before the next event
            Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
        }
    }
}

/// GO button: short press → `Go`, long press (≥800 ms) → `GoLong`.
#[embassy_executor::task]
pub async fn go_task(mut pin: Input<'static>) {
    loop {
        pin.wait_for_falling_edge().await;
        Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
        if pin.is_low() {
            let t = Instant::now();
            pin.wait_for_high().await;
            let ev = if t.elapsed() >= LONG_PRESS { Event::GoLong } else { Event::Go };
            EVENTS.send(ev).await;
            Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
        }
    }
}

/// Await the next event while keeping the watchdog fed during idle waits.
/// Every blocking wait in the game must go through here so a legitimately-idle
/// machine (waiting for a player) doesn't trip the watchdog. See IMPLEMENTATION.md §2.1.
pub async fn recv(wdt: &mut Watchdog) -> Event {
    loop {
        wdt.feed();
        match select(EVENTS.receive(), Timer::after(Duration::from_millis(500))).await {
            Either::First(ev) => return ev,
            Either::Second(()) => {} // heartbeat tick — feed and keep waiting
        }
    }
}
