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
use libm::{powf, sinf};
use smart_leds::RGB8;

use crate::config::{COUNTS, FRAME_MS, LANES, NUM_LEDS, ROWS};

/// What the renderer should draw. Produced by the game task, consumed by `led_task`.
#[derive(Clone, Copy)]
pub enum Mode {
    Home,
    Attract,
    Select(u8),
    Race,
    Winner(u8),
}

#[derive(Clone, Copy)]
pub struct RaceView {
    pub mode: Mode,
    pub progress: [f32; LANES],
}

impl Default for RaceView {
    fn default() -> Self {
        Self { mode: Mode::Home, progress: [0.0; LANES] }
    }
}

/// Latest render state. `led_task` polls this each frame (never blocks on it).
pub static RACE_VIEW: Signal<CriticalSectionRawMutex, RaceView> = Signal::new();

/// Distinct colour per duck (indexed by lane).
pub const DUCK_COLORS: [RGB8; LANES] = [
    RGB8 { r: 255, g: 40, b: 0 },  // red-orange
    RGB8 { r: 0, g: 120, b: 255 }, // blue
    RGB8 { r: 0, g: 220, b: 40 },  // green
    RGB8 { r: 235, g: 190, b: 0 }, // yellow
];

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

/// Map a progress fraction (0=start, 1=finish) to a position within row `r`.
fn pos(r: usize, f: f32) -> usize {
    let f = f.clamp(0.0, 1.0);
    (f * (COUNTS[r] - 1) as f32 + 0.5) as usize
}

pub struct LedController<'d> {
    ws: PioWs2812<'d, PIO0, 0, NUM_LEDS, Grb>,
    fb: [RGB8; NUM_LEDS],
    gamma: [u8; 256],
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
        Self {
            ws: PioWs2812::new(common, sm, dma, pin, program),
            fb: [RGB8::default(); NUM_LEDS],
            gamma,
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

    /// Draw a comet at progress `f` in `lane`'s row, with a short fading tail.
    /// (`max_px` here only merges the comet's own overlapping tail — the shared-column
    /// compositing the top-down layout needed is gone now that lane↔row is 1:1.)
    fn draw_comet(&mut self, lane: usize, f: f32, color: RGB8) {
        const TAIL: usize = 3;
        let head = pos(lane, f);
        for k in 0..=TAIL {
            if head >= k {
                let bright = 1.0 - (k as f32 / (TAIL as f32 + 1.0));
                let idx = phys_index(lane, head - k);
                let c = self.scaled(color, bright);
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

    /// Render the current view and push it to the strip. Called every ~FRAME_MS.
    pub async fn render(&mut self, view: &RaceView) {
        self.t += FRAME_MS as f32 / 1000.0;
        if self.t > 3600.0 {
            self.t -= 3600.0;
        }
        self.fb = [RGB8::default(); NUM_LEDS];

        match view.mode {
            Mode::Home => {
                // Dim amber breathing while returning to home.
                let b = 0.15 + 0.1 * (0.5 + 0.5 * sinf(self.t * 3.0));
                for l in 0..LANES {
                    self.fill_lane(l, RGB8 { r: 255, g: 120, b: 0 }, b);
                }
            }
            Mode::Attract => {
                // Slow travelling shimmer across all lanes in each duck's colour.
                for l in 0..LANES {
                    let b = 0.10 + 0.35 * (0.5 + 0.5 * sinf(self.t * 2.0 + l as f32 * 1.3));
                    self.fill_lane(l, DUCK_COLORS[l], b);
                }
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
                for l in 0..LANES {
                    self.draw_comet(l, view.progress[l], DUCK_COLORS[l]);
                }
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
    let mut view = RaceView::default();
    let mut ticker = Ticker::every(Duration::from_millis(FRAME_MS));
    loop {
        if let Some(v) = RACE_VIEW.try_take() {
            view = v;
        }
        leds.render(&view).await;
        ticker.next().await;
    }
}
