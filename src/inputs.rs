//! Buttons and limit switches → debounced events on a shared channel.
//!
//! All inputs are active-low with internal pull-ups (press / switch-close = falling
//! edge). One `input_task` per pin; GO gets its own task so it can distinguish a short
//! press (start race) from a long press (save & exit TUNE). See IMPLEMENTATION.md §7.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_futures::select::{select, Either};
use embassy_rp::gpio::Input;
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};

use crate::config::{DEBOUNCE_MS, LANES};

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

// ---- Limit-switch LEVEL cache ---------------------------------------------
// Events are edge-triggered, which makes "the switch is already closed" invisible: a
// gantry resting on its home switch at power-up never produces a falling edge, so an
// events-only wait sits there until the timeout. These mirror the *current* debounced
// state of each limit switch so callers can ask "is this lane home?" instead of only
// "did this lane just arrive?". `true` = switch closed (pin low).
//
// Load/store on `AtomicBool` (no compare-exchange) is natively available on the M0+.
static HOME_CLOSED: [AtomicBool; LANES] = [const { AtomicBool::new(false) }; LANES];
static END_CLOSED: [AtomicBool; LANES] = [const { AtomicBool::new(false) }; LANES];

/// Which level slot (if any) an event feeds. Buttons have no level cache.
fn level_slot(ev: Event) -> Option<&'static AtomicBool> {
    match ev {
        Event::StartHit(l) => HOME_CLOSED.get(l as usize),
        Event::EndHit(l) => END_CLOSED.get(l as usize),
        _ => None,
    }
}

/// Is lane `l` currently sitting on its home (start) switch?
pub fn home_closed(l: usize) -> bool {
    HOME_CLOSED.get(l).is_some_and(|a| a.load(Ordering::Relaxed))
}

/// Is lane `l` currently sitting on its finish switch?
pub fn end_closed(l: usize) -> bool {
    END_CLOSED.get(l).is_some_and(|a| a.load(Ordering::Relaxed))
}

/// Discard any queued events. Call before a phase that must not act on a stale edge —
/// notably race start, where a leftover `EndHit` would score an instant false win.
pub fn drain() {
    while EVENTS.try_receive().is_ok() {}
}

/// Generic input: emit a fixed event on each debounced press, one event per press,
/// and keep the limit-switch level cache in step with the pin.
#[embassy_executor::task(pool_size = 16)]
pub async fn input_task(mut pin: Input<'static>, ev: Event) {
    let slot = level_slot(ev);
    // Seed from the pin's ACTUAL state before waiting on any edge — this is what makes
    // an already-closed switch visible at boot.
    if let Some(s) = slot {
        s.store(pin.is_low(), Ordering::Relaxed);
    }
    loop {
        pin.wait_for_falling_edge().await;
        Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
        if pin.is_low() {
            // Set the level before sending: `send` can block on a full channel, and the
            // level must never lag the physical switch.
            if let Some(s) = slot {
                s.store(true, Ordering::Relaxed);
            }
            EVENTS.send(ev).await;
            pin.wait_for_high().await; // require release before the next event
            if let Some(s) = slot {
                s.store(false, Ordering::Relaxed);
            }
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
