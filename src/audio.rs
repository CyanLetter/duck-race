//! Audio — stubbed. No hardware yet; the game calls `play(..)` at the trigger points
//! so a real backend drops in later without touching game logic. See IMPLEMENTATION.md §9.
//!
//! Planned backend: DFPlayer Mini on UART0 (GP0 TX / GP1 RX, 9600 baud), microSD clips,
//! 3 W speaker, volume via DFPlayer command and/or a pot on GP28.

#[allow(dead_code)]
#[derive(Clone, Copy, defmt::Format)]
pub enum Sound {
    Bet,     // a duck was selected
    Race,    // race started
    Finish,  // first duck crossed the line
    Attract, // idle
    Home,    // homing/reset
}

pub trait AudioSink {
    fn play(&mut self, s: Sound);
    #[allow(dead_code)] // wired for the future DFPlayer backend / volume pot
    fn set_volume(&mut self, v: u8);
}

/// Current backend: logs the trigger over RTT and does nothing else.
pub struct NullAudio;

impl AudioSink for NullAudio {
    fn play(&mut self, s: Sound) {
        defmt::debug!("audio: {}", s);
    }
    fn set_volume(&mut self, _v: u8) {}
}

// Later: `struct DfPlayer<'d> { uart: Uart<'d, Blocking>, volume: u8 }` building the
// 10-byte `7E FF 06 <cmd> 00 <hi> <lo> <cksumHi> <cksumLo> EF` command frames.
