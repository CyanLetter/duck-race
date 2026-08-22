//! Buttons and limit switches → debounced events on a shared channel.
//!
//! All inputs are active-low with internal pull-ups (press / switch-close = falling
//! edge). One `input_task` per pin; GO gets its own task so it can distinguish a short
//! press (start race) from a long press (save & exit TUNE). See IMPLEMENTATION.md §7.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_executor::Spawner;
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

/// Seed the level cache from the pin, then hand it to `input_task`.
///
/// The seed happens **here**, synchronously in `main` before any `.await`, rather than on
/// the task's first poll. Otherwise whether the very first `home()` sees a switch that was
/// already closed at power-up depends on whether anything happened to yield to the
/// executor between spawning and homing — correct by accident, and silently broken by
/// reordering `main`.
pub fn spawn_input(spawner: Spawner, pin: Input<'static>, ev: Event) {
    if let Some(s) = level_slot(ev) {
        s.store(pin.is_low(), Ordering::Relaxed);
    }
    spawner.must_spawn(input_task(pin, ev));
}

/// Generic input: emit a fixed event on each debounced press, one event per press,
/// and keep the limit-switch level cache in step with the pin at all times.
///
/// **Waits on levels, not edges.** `wait_for_high`/`wait_for_low` return immediately if
/// the pin is already in that state, so by always waiting for the *opposite* of where we
/// are, the task can never be blind to a state that was already true when it started —
/// and, critically, it always gets to run again when the pin changes back.
///
/// The previous edge-based version (`wait_for_falling_edge` → debounce → `wait_for_high`)
/// had a latch bug: a switch closed at boot parked the task in `wait_for_falling_edge`
/// forever, because an already-low pin never produces a falling edge. The `store(false)`
/// on release sat downstream of that wait and was unreachable, so once seeded `true` the
/// level stayed `true` even after the gantry left home — and `home()` would then skip
/// homing entirely on the "all lanes already home" path.
#[embassy_executor::task(pool_size = 16)]
pub async fn input_task(mut pin: Input<'static>, ev: Event) {
    let slot = level_slot(ev);
    let mut closed = pin.is_low();
    if let Some(s) = slot {
        s.store(closed, Ordering::Relaxed);
    }
    loop {
        if closed {
            pin.wait_for_high().await;
        } else {
            pin.wait_for_low().await;
        }
        Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
        let settled = pin.is_low();
        if settled == closed {
            continue; // bounce that landed back where it started — nothing happened
        }
        closed = settled;
        // Update the level before sending: `send` can block on a full channel, and the
        // level must never lag the physical switch.
        if let Some(s) = slot {
            s.store(closed, Ordering::Relaxed);
        }
        if closed {
            EVENTS.send(ev).await; // one event per press, on close only
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
