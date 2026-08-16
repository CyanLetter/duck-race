//! Audio — DY-SV8F voice module on UART0, playing from its onboard 8 MB flash.
//! See IMPLEMENTATION.md §9 and Reference/DY-SV8F.pdf.
//!
//! Wiring: Pico **GP0 (UART0 TX) → module RXD/IO1 (pin 4)**, GND common. The module's
//! TXD (pin 3) can go to GP1 if status queries are ever wanted — this driver is TX-only,
//! which is all the game needs. Module DIP switches must be in **UART mode**.
//!
//! Protocol: 9600 8N1, frames are `AA <cmd> <len> <data…> <sum>` where `sum` is the low
//! byte of the arithmetic sum of every preceding byte. (Note this is NOT the DFPlayer's
//! `7E…EF` format — different module family entirely.)
//!
//! # Track map
//!
//! The DY-SV8F selects a track by the **number in its 5-digit filename**, so the clips
//! must be named exactly:
//!
//! | File        | Sound          | Played when                          |
//! |-------------|----------------|--------------------------------------|
//! | `00001.mp3` | `Bet(0)`       | duck 0 selected                      |
//! | `00002.mp3` | `Bet(1)`       | duck 1 selected                      |
//! | `00003.mp3` | `Bet(2)`       | duck 2 selected                      |
//! | `00004.mp3` | `Bet(3)`       | duck 3 selected                      |
//! | `00005.mp3` | `Race`         | race starts                          |
//! | `00006.mp3` | `Win`          | the player's duck won                |
//! | `00007.mp3` | `Lose`         | a different duck won                 |
//!
//! Copy the files onto the module's flash **in numeric order** anyway — it costs nothing
//! and sidesteps the DY-family quirk where some firmware indexes by write order rather
//! than by filename.

use embassy_rp::uart::{Blocking, UartTx};
use embassy_time::{Duration, Timer};

use crate::config::LANES;

// ---- Protocol constants (Reference/DY-SV8F.pdf, "UART Communication Command") --------
const FRAME_START: u8 = 0xAA;
const CMD_STOP: u8 = 0x04;
const CMD_SELECT_DRIVE: u8 = 0x0B;
const CMD_PLAY_TRACK: u8 = 0x07; // "Specified Song", 16-bit track number
const CMD_SET_VOLUME: u8 = 0x13; // 0..=30
const CMD_SET_LOOP: u8 = 0x18;
const DRIVE_FLASH: u8 = 0x02; // USB=00, SD=01, FLASH=02
const LOOP_SINGLE_STOP: u8 = 0x02; // play the selected track once, then stop
/// Module's volume scale is 0..=30 (datasheet: "31 grades", default 20).
pub const VOLUME_MAX: u8 = 30;
pub const VOLUME_DEFAULT: u8 = 22;

#[allow(dead_code)]
#[derive(Clone, Copy, defmt::Format)]
pub enum Sound {
    Bet(u8), // a duck was selected — one clip per lane
    Race,    // race started
    Win,     // the player's duck won
    Lose,    // some other duck won
    Attract, // idle — no clip assigned yet
    Home,    // homing/reset — no clip assigned yet
}

impl Sound {
    /// Track number (= the number in the 5-digit filename), or `None` for triggers that
    /// have no clip yet. Adding a clip later is a one-line change here.
    pub fn track(self) -> Option<u16> {
        match self {
            Sound::Bet(l) => Some(1 + (l as u16).min(LANES as u16 - 1)),
            Sound::Race => Some(5),
            Sound::Win => Some(6),
            Sound::Lose => Some(7),
            Sound::Attract | Sound::Home => None,
        }
    }
}

pub trait AudioSink {
    fn play(&mut self, s: Sound);
    #[allow(dead_code)] // used by the bring-up audio test
    fn set_volume(&mut self, v: u8);
}

/// Fallback backend: logs the trigger over RTT and does nothing else. Kept so the game
/// can be built and run with no audio hardware attached — swap it in for `DySv8f` in
/// `main.rs` if the module is unplugged.
#[allow(dead_code)]
pub struct NullAudio;

impl AudioSink for NullAudio {
    fn play(&mut self, s: Sound) {
        defmt::debug!("audio (null): {}", s);
    }
    fn set_volume(&mut self, _v: u8) {}
}

/// DY-SV8F over UART, TX-only.
pub struct DySv8f<'d> {
    tx: UartTx<'d, Blocking>,
    volume: u8,
}

impl<'d> DySv8f<'d> {
    pub fn new(tx: UartTx<'d, Blocking>) -> Self {
        Self { tx, volume: VOLUME_DEFAULT }
    }

    /// Build and send one command frame. Writes go into the UART's 32-byte TX FIFO, so
    /// `blocking_write` returns immediately for frames this small — it does not stall the
    /// executor (which would starve the LED renderer; see §2.1).
    fn send(&mut self, cmd: u8, data: &[u8]) {
        let mut buf = [0u8; 8];
        buf[0] = FRAME_START;
        buf[1] = cmd;
        buf[2] = data.len() as u8;
        let n = 3 + data.len();
        buf[3..n].copy_from_slice(data);
        buf[n] = buf[..n].iter().fold(0u8, |a, b| a.wrapping_add(*b));
        if self.tx.blocking_write(&buf[..=n]).is_err() {
            defmt::warn!("audio: UART write failed (cmd {=u8:#04x})", cmd);
        }
    }

    /// Point the module at its onboard flash, set a known play mode and volume.
    /// Call once at startup — the module needs a moment after power-up before it will
    /// accept commands.
    pub async fn init(&mut self) {
        Timer::after(Duration::from_millis(500)).await;
        self.send(CMD_SELECT_DRIVE, &[DRIVE_FLASH]);
        Timer::after(Duration::from_millis(200)).await; // mounting the flash takes a beat
        self.send(CMD_SET_LOOP, &[LOOP_SINGLE_STOP]);
        let v = self.volume;
        self.send(CMD_SET_VOLUME, &[v]);
        defmt::info!("audio: DY-SV8F init (FLASH, single-stop, volume {}/{})", v, VOLUME_MAX);
    }

    /// Play a track by its filename number (`3` → `00003.mp3`).
    pub fn play_track(&mut self, n: u16) {
        self.send(CMD_PLAY_TRACK, &[(n >> 8) as u8, n as u8]);
    }

    #[allow(dead_code)]
    pub fn stop(&mut self) {
        self.send(CMD_STOP, &[]);
    }

    #[allow(dead_code)]
    pub fn volume(&self) -> u8 {
        self.volume
    }
}

impl AudioSink for DySv8f<'_> {
    fn play(&mut self, s: Sound) {
        match s.track() {
            Some(n) => {
                defmt::debug!("audio: {} -> track {}", s, n);
                self.play_track(n);
            }
            None => defmt::debug!("audio: {} (no clip assigned)", s),
        }
    }

    fn set_volume(&mut self, v: u8) {
        self.volume = v.min(VOLUME_MAX);
        let v = self.volume;
        self.send(CMD_SET_VOLUME, &[v]);
    }
}
