//! WS2812 rendering: 4 serpentine rows, lane/progress space, decoupled renderer.
//! Uses the built-in `embassy_rp::pio_programs::ws2812` driver. See IMPLEMENTATION.md §5.
//!
//! Side-on cabinet: lanes are stacked vertically and **each lane owns one row** — `lane
//! == row`, nothing shared. Row r is wired FORWARD (start→finish) if r is even, REVERSED
//! if r is odd; `phys_index` absorbs that flip so all higher-level code works in
//! (lane, progress) space.
//!
//! The chain order must match the physical top-to-bottom lane order AND the duck-button
//! order — verify with `test_walk` (feature `test-leds`) before wiring the panel.

use embassy_rp::Peri;
use embassy_rp::peripherals::{DMA_CH0, PIN_22, PIO0};
use embassy_rp::pio::{Common, StateMachine};
use embassy_rp::pio_programs::ws2812::{Grb, PioWs2812, PioWs2812Program};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use libm::{expf, powf, sinf};
use smart_leds::RGB8;

use crate::config::{
    CHASE_SPACING, CHASE_STEP_MS_ATTRACT, CHASE_STEP_MS_RACE, CHASE_TAU_STEPS, COUNTS,
    FRAME_MS, LANES, NUM_LEDS, ROWS,
};

/// What the renderer should draw. Produced by the game task, consumed by `led_task`.
///
/// Note there is no per-lane progress here: during a race every row runs the same marquee
/// chase rather than tracking where its duck actually is. The ducks are visible in profile
/// in a side-on cabinet, so the lights are set dressing, not a readout.
#[derive(Clone, Copy)]
pub enum Mode {
    Home,
    Attract,
    Select(u8),
    Race,
    Winner(u8),
}

/// Latest render state. `led_task` polls this each frame (never blocks on it).
pub static RACE_VIEW: Signal<CriticalSectionRawMutex, Mode> = Signal::new();

/// Distinct colour per duck (indexed by lane).
pub const DUCK_COLORS: [RGB8; LANES] = [
    RGB8 { r: 255, g: 40, b: 0 },  // red-orange
    RGB8 { r: 0, g: 120, b: 255 }, // blue
    RGB8 { r: 0, g: 220, b: 40 },  // green
    RGB8 { r: 235, g: 190, b: 0 }, // yellow
];

/// Warm-white filament colour for the marquee chase. Pre-gamma — after the 2.2 LUT this
/// lands near a ~2700 K incandescent rather than a cold white. To chase in each lane's own
/// colour instead, pass `DUCK_COLORS[row]` inside `draw_chase`.
// pub const CHASE_COLOR: RGB8 = RGB8 { r: 255, g: 180, b: 100 };
pub const CHASE_COLOR: RGB8 = RGB8 { r: 150, g: 90, b: 50 };

/// Resolution of the bulb-decay curve. Indexed by "age since this pixel was the bulb",
/// scaled over one full `CHASE_SPACING` interval.
const CHASE_LUT_LEN: usize = 128;

/// Prefix sums of COUNTS → each row's start index in the chain.
const fn offsets() -> [usize; ROWS] {
    let mut o = [0usize; ROWS];
    let mut i = 1;
    while i < ROWS {
        o[i] = o[i - 1] + COUNTS[i - 1];
        i += 1;
    }
    o
}
const OFFSET: [usize; ROWS] = offsets();

/// Physical chain index for row `r` at position `p` measured FROM THE START.
fn phys_index(r: usize, p: usize) -> usize {
    if r % 2 == 0 {
        OFFSET[r] + p // forward-wired row
    } else {
        OFFSET[r] + (COUNTS[r] - 1 - p) // reversed row
    }
}

pub struct LedController<'d> {
    ws: PioWs2812<'d, PIO0, 0, NUM_LEDS, Grb>,
    fb: [RGB8; NUM_LEDS],
    gamma: [u8; 256],
    chase: [u8; CHASE_LUT_LEN],
    t: f32,
}

impl<'d> LedController<'d> {
    pub fn new(
        common: &mut Common<'d, PIO0>,
        sm: StateMachine<'d, PIO0, 0>,
        dma: Peri<'d, DMA_CH0>,
        pin: Peri<'d, PIN_22>,
        program: &PioWs2812Program<'d, PIO0>,
    ) -> Self {
        // Precompute a gamma LUT once (keeps powf out of the per-frame hot path — the
        // wasteful pattern the reference had; see IMPLEMENTATION.md §2.1).
        let mut gamma = [0u8; 256];
        for (i, g) in gamma.iter_mut().enumerate() {
            *g = (powf(i as f32 / 255.0, 2.2) * 255.0) as u8;
        }
        // Bulb decay curve, also precomputed: `expf` per pixel per frame would be the
        // same soft-float mistake the reference project made with `powf` (§2.1).
        let mut chase = [0u8; CHASE_LUT_LEN];
        for (i, c) in chase.iter_mut().enumerate() {
            let age = i as f32 / CHASE_LUT_LEN as f32 * CHASE_SPACING as f32;
            *c = (expf(-age / CHASE_TAU_STEPS) * 255.0) as u8;
        }
        Self {
            ws: PioWs2812::new(common, sm, dma, pin, program),
            fb: [RGB8::default(); NUM_LEDS],
            gamma,
            chase,
            t: 0.0,
        }
    }

    fn scaled(&self, c: RGB8, b: f32) -> RGB8 {
        let b = b.clamp(0.0, 1.0);
        RGB8 {
            r: self.gamma[(c.r as f32 * b) as usize],
            g: self.gamma[(c.g as f32 * b) as usize],
            b: self.gamma[(c.b as f32 * b) as usize],
        }
    }

    fn max_px(fb: &mut [RGB8; NUM_LEDS], idx: usize, c: RGB8) {
        let p = &mut fb[idx];
        p.r = p.r.max(c.r);
        p.g = p.g.max(c.g);
        p.b = p.b.max(c.b);
    }

    /// Carnival-marquee chase, drawn identically on every row and in sync across rows so
    /// the whole board reads as one sign.
    ///
    /// A bulb lights at every `CHASE_SPACING`-th pixel and the whole set advances one
    /// pixel every `step_ms`, travelling start → finish. Each bulb snaps to full and then
    /// decays exponentially, which is what makes it look like a hot filament cooling
    /// rather than an LED switching off.
    ///
    /// Stateless: brightness is a pure function of "how long since this pixel was last a
    /// bulb", so there's no per-pixel decay buffer to keep in step with the frame rate.
    fn draw_chase(&mut self, step_ms: u64, color: RGB8) {
        let span = CHASE_SPACING as f32;
        // Chase position in pixel-steps. Fractional, so the fade is smooth between steps
        // instead of the whole pattern jumping once per step.
        let phase = self.t * 1000.0 / step_ms as f32;
        for row in 0..ROWS {
            for p in 0..COUNTS[row] {
                // Steps elapsed since this pixel was the bulb, wrapped into [0, spacing).
                let mut age = (phase - p as f32) % span;
                if age < 0.0 {
                    age += span;
                }
                let lut = ((age / span) * CHASE_LUT_LEN as f32) as usize;
                let bright = self.chase[lut.min(CHASE_LUT_LEN - 1)] as f32 / 255.0;
                let c = self.scaled(color, bright);
                let idx = phys_index(row, p);
                Self::max_px(&mut self.fb, idx, c);
            }
        }
    }

    fn fill_lane(&mut self, lane: usize, color: RGB8, bright: f32) {
        let c = self.scaled(color, bright);
        for p in 0..COUNTS[lane] {
            let idx = phys_index(lane, p);
            Self::max_px(&mut self.fb, idx, c);
        }
    }

    /// Render the current mode and push it to the strip. Called every ~FRAME_MS.
    pub async fn render(&mut self, mode: Mode) {
        self.t += FRAME_MS as f32 / 1000.0;
        // Wrap on a whole number of ATTRACT chase periods (period = SPACING × step). The
        // chase phase is `t / step % SPACING`, so subtracting an exact multiple of the
        // period leaves the pattern untouched — no jump when the clock rolls over. Attract
        // is the mode that actually runs for hours; a race is ~12 s, so the odds of it
        // straddling the wrap are negligible and the artefact would be under one step.
        let chase_period = CHASE_SPACING as f32 * CHASE_STEP_MS_ATTRACT as f32 / 1000.0;
        if self.t > 3600.0 {
            self.t -= chase_period * (3600.0 / chase_period) as i32 as f32;
        }
        self.fb = [RGB8::default(); NUM_LEDS];

        match mode {
            Mode::Home => {
                // Dim amber breathing while returning to home.
                let b = 0.15 + 0.1 * (0.5 + 0.5 * sinf(self.t * 3.0));
                for l in 0..LANES {
                    self.fill_lane(l, RGB8 { r: 255, g: 120, b: 0 }, b);
                }
            }
            Mode::Attract => {
                // The same marquee as the race, just idling along slower.
                self.draw_chase(CHASE_STEP_MS_ATTRACT, CHASE_COLOR);
            }
            Mode::Select(sel) => {
                let sel = sel as usize;
                for l in 0..LANES {
                    if l == sel {
                        let b = 0.4 + 0.6 * (0.5 + 0.5 * sinf(self.t * 6.0));
                        self.fill_lane(l, DUCK_COLORS[l], b);
                    } else {
                        self.fill_lane(l, DUCK_COLORS[l], 0.06); // others dim
                    }
                }
            }
            Mode::Race => {
                // Marquee at full speed. Deliberately NOT a position readout — this runs
                // from the moment GO is pressed, through the start delay, to the finish.
                self.draw_chase(CHASE_STEP_MS_RACE, CHASE_COLOR);
            }
            Mode::Winner(w) => {
                let w = w as usize;
                let blink = if sinf(self.t * 12.0) > 0.0 { 1.0 } else { 0.15 };
                self.fill_lane(w, DUCK_COLORS[w], blink);
            }
        }

        self.ws.write(&self.fb).await;
    }

    /// BRING-UP: verify serpentine wiring, direction, per-row counts, and row ORDER.
    /// Pass 1 walks a single dot through the physical chain (watch it snake). Pass 2
    /// lights each row in its duck's colour — **check that row 0 is the same lane as
    /// duck button 0, top to bottom, before the panel loom is soldered**. Pass 3 walks a
    /// dot along each row from its START end (should always begin at the start, proving
    /// `phys_index` compensates for the reversed odd rows). Loops forever.
    #[cfg(feature = "test-leds")]
    pub async fn test_walk(&mut self, wdt: &mut embassy_rp::watchdog::Watchdog) -> ! {
        use embassy_time::{Duration, Timer};
        // Row colours ARE the duck colours, dimmed — that's what makes pass 2 a valid
        // check of the lane ↔ row ↔ button ordering.
        let cols: [RGB8; ROWS] = core::array::from_fn(|r| RGB8 {
            r: DUCK_COLORS[r].r / 3,
            g: DUCK_COLORS[r].g / 3,
            b: DUCK_COLORS[r].b / 3,
        });
        defmt::info!(
            "BRINGUP LED test: pass1 dot walks chain, pass2 rows in duck colours, pass3 per-row start->finish"
        );
        loop {
            // Pass 1 — one dot through the physical chain.
            for i in 0..NUM_LEDS {
                self.fb = [RGB8::default(); NUM_LEDS];
                self.fb[i] = RGB8 { r: 60, g: 60, b: 60 };
                self.ws.write(&self.fb).await;
                wdt.feed();
                Timer::after(Duration::from_millis(45)).await;
            }
            // Pass 2 — each row in its duck's colour, held ~2 s.
            self.fb = [RGB8::default(); NUM_LEDS];
            for r in 0..ROWS {
                for p in 0..COUNTS[r] {
                    self.fb[phys_index(r, p)] = cols[r];
                }
            }
            self.ws.write(&self.fb).await;
            for _ in 0..40 {
                wdt.feed();
                Timer::after(Duration::from_millis(50)).await;
            }
            // Pass 3 — per row, dot from START(pos 0) to finish.
            for r in 0..ROWS {
                for p in 0..COUNTS[r] {
                    self.fb = [RGB8::default(); NUM_LEDS];
                    self.fb[phys_index(r, p)] = cols[r];
                    self.ws.write(&self.fb).await;
                    wdt.feed();
                    Timer::after(Duration::from_millis(40)).await;
                }
            }
        }
    }
}

/// Decoupled renderer: fixed frame tick, reads the latest `RaceView`, never blocked by
/// game logic. (This is why animation stays smooth where the reference stalled — §2.1.)
#[embassy_executor::task]
pub async fn led_task(mut leds: LedController<'static>) {
    use embassy_time::{Duration, Ticker};
    let mut mode = Mode::Home;
    let mut ticker = Ticker::every(Duration::from_millis(FRAME_MS));
    loop {
        if let Some(m) = RACE_VIEW.try_take() {
            mode = m;
        }
        leds.render(mode).await;
        ticker.next().await;
    }
}
