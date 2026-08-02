//! Per-lane motor baseline calibration + flash persistence. See IMPLEMENTATION.md §6.
//!
//! Enter TUNE by holding GO at power-up. Then: duck-select picks the active lane,
//! UP/DOWN nudge its baseline duty, GO runs a test race on that lane, long-GO saves all
//! baselines to the last flash sector and exits. Flash is written ONLY on save (§2.1).

use embassy_rp::Peri;
use embassy_rp::flash::{Blocking, Flash};
use embassy_rp::peripherals::FLASH;
use embassy_rp::watchdog::Watchdog;
use embassy_time::{with_timeout, Duration, Instant};

use crate::config::{
    BASE_DEFAULT_PCT, FLOOR_PCT, HOMING_PCT, LANES, RACE_TIMEOUT_MS, RESET_TIMEOUT_MS,
};
use crate::inputs::{recv, Event, EVENTS};
use crate::leds::{Mode, RaceView, RACE_VIEW};
use crate::motors::Motors;

pub const FLASH_SIZE: usize = 2 * 1024 * 1024;
pub type CalFlash = Flash<'static, FLASH, Blocking, FLASH_SIZE>;

const CFG_OFFSET: u32 = (FLASH_SIZE as u32) - 4096; // last 4 KB sector
const MAGIC: u32 = 0xD0CC_0001;

pub fn new_flash(f: Peri<'static, FLASH>) -> CalFlash {
    Flash::new_blocking(f)
}

#[derive(Clone, Copy)]
pub struct Baselines {
    pub pct: [u8; LANES],
}

impl Default for Baselines {
    fn default() -> Self {
        Self { pct: [BASE_DEFAULT_PCT; LANES] }
    }
}

fn checksum(pct: &[u8; 4]) -> u32 {
    MAGIC ^ (pct[0] as u32)
        ^ ((pct[1] as u32) << 8)
        ^ ((pct[2] as u32) << 16)
        ^ ((pct[3] as u32) << 24)
}

/// Load baselines from flash, falling back to defaults if unwritten/invalid.
pub fn load(flash: &mut CalFlash) -> Baselines {
    let mut buf = [0u8; 12];
    if flash.blocking_read(CFG_OFFSET, &mut buf).is_ok() {
        let magic = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        let pct = [buf[4], buf[5], buf[6], buf[7]];
        let crc = u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
        let sane = pct.iter().all(|&p| p >= FLOOR_PCT && p <= 100);
        if magic == MAGIC && crc == checksum(&pct) && sane {
            defmt::info!("loaded baselines {}", pct);
            return Baselines { pct };
        }
    }
    defmt::info!("no valid baselines in flash — using defaults");
    Baselines::default()
}

/// Erase the config sector and write the baselines (only called on explicit save).
pub fn save(flash: &mut CalFlash, b: &Baselines) {
    let mut buf = [0u8; 12];
    buf[0..4].copy_from_slice(&MAGIC.to_le_bytes());
    buf[4..8].copy_from_slice(&b.pct);
    buf[8..12].copy_from_slice(&checksum(&b.pct).to_le_bytes());
    if flash.blocking_erase(CFG_OFFSET, CFG_OFFSET + 4096).is_ok()
        && flash.blocking_write(CFG_OFFSET, &buf).is_ok()
    {
        defmt::info!("saved baselines {}", b.pct);
    } else {
        defmt::error!("flash save failed");
    }
}

/// Blocking TUNE-mode loop. Returns (and normal game starts) on long-GO save.
pub async fn tune_mode(
    motors: &mut Motors<'_>,
    wdt: &mut Watchdog,
    flash: &mut CalFlash,
    base: &mut Baselines,
) {
    defmt::info!("TUNE mode — select lane, UP/DOWN adjust, GO test, hold-GO save+exit");
    let mut active = 0usize;
    RACE_VIEW.signal(RaceView { mode: Mode::Select(active as u8), progress: [0.0; LANES] });

    loop {
        match recv(wdt).await {
            Event::Select(d) => {
                active = (d as usize).min(LANES - 1);
                RACE_VIEW.signal(RaceView { mode: Mode::Select(active as u8), progress: [0.0; LANES] });
            }
            Event::Up => {
                base.pct[active] = (base.pct[active] + 2).min(100);
                defmt::info!("lane {} baseline -> {}%", active, base.pct[active]);
            }
            Event::Down => {
                base.pct[active] = base.pct[active].saturating_sub(2).max(FLOOR_PCT);
                defmt::info!("lane {} baseline -> {}%", active, base.pct[active]);
            }
            Event::Go => test_lane(motors, wdt, active, base.pct[active]).await,
            Event::GoLong => {
                save(flash, base);
                return;
            }
            _ => {}
        }
    }
}

/// Run one lane forward to its finish switch, then home it — so you can eyeball speed.
async fn test_lane(motors: &mut Motors<'_>, wdt: &mut Watchdog, lane: usize, pct: u8) {
    motors.enable(true);
    motors.set_lane_forward(lane, pct);
    let start = Instant::now();
    loop {
        wdt.feed();
        if let Ok(Event::EndHit(l)) =
            with_timeout(Duration::from_millis(200), EVENTS.receive()).await
        {
            if l as usize == lane {
                break;
            }
        }
        if start.elapsed() > Duration::from_millis(RACE_TIMEOUT_MS) {
            break;
        }
    }
    // Home this lane (shared reverse drives all, but the others are already home).
    motors.reverse_all(HOMING_PCT);
    let start = Instant::now();
    loop {
        wdt.feed();
        if let Ok(Event::StartHit(l)) =
            with_timeout(Duration::from_millis(200), EVENTS.receive()).await
        {
            if l as usize == lane {
                break;
            }
        }
        if start.elapsed() > Duration::from_millis(RESET_TIMEOUT_MS) {
            break;
        }
    }
    motors.coast_all();
}
