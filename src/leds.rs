//! WS2812 rendering: 5 serpentine columns, lane/progress space, decoupled renderer.
//! Uses the built-in `embassy_rp::pio_programs::ws2812` driver. See IMPLEMENTATION.md §5.
//!
//! Column c is wired FORWARD (start→finish) if c is even, REVERSED if c is odd. Lane i
//! is flanked by columns i (left) and i+1 (right); interior columns are shared by two
//! lanes and composited per-pixel with max. `phys_index` absorbs the wiring direction so
//! all higher-level code works in (lane, progress) space.

use embassy_rp::Peri;
use embassy_rp::peripherals::{DMA_CH0, PIN_9, PIO0};
use embassy_rp::pio::{Common, StateMachine};
use embassy_rp::pio_programs::ws2812::{Grb, PioWs2812, PioWs2812Program};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use libm::{powf, sinf};
use smart_leds::RGB8;

use crate::config::{COUNTS, FRAME_MS, LANES, NUM_LEDS};

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

/// Prefix sums of COUNTS → each column's start index in the chain.
const fn offsets() -> [usize; 5] {
    let mut o = [0usize; 5];
    let mut i = 1;
    while i < 5 {
        o[i] = o[i - 1] + COUNTS[i - 1];
        i += 1;
    }
    o
}
const OFFSET: [usize; 5] = offsets();

/// Physical chain index for column `c` at position `p` measured FROM THE START.
fn phys_index(c: usize, p: usize) -> usize {
    if c % 2 == 0 {
        OFFSET[c] + p // forward-wired column
    } else {
        OFFSET[c] + (COUNTS[c] - 1 - p) // reversed column
    }
}

/// Map a progress fraction (0=start, 1=finish) to a position within column `c`.
fn pos(c: usize, f: f32) -> usize {
    let f = f.clamp(0.0, 1.0);
    (f * (COUNTS[c] - 1) as f32 + 0.5) as usize
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
        pin: Peri<'d, PIN_9>,
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

    /// Draw a comet at progress `f` on both columns flanking `lane`, with a short tail.
    fn draw_comet(&mut self, lane: usize, f: f32, color: RGB8) {
        const TAIL: usize = 3;
        for col in [lane, lane + 1] {
            let head = pos(col, f);
            for k in 0..=TAIL {
                if head >= k {
                    let bright = 1.0 - (k as f32 / (TAIL as f32 + 1.0));
                    let idx = phys_index(col, head - k);
                    let c = self.scaled(color, bright);
                    Self::max_px(&mut self.fb, idx, c);
                }
            }
        }
    }

    fn fill_lane(&mut self, lane: usize, color: RGB8, bright: f32) {
        for col in [lane, lane + 1] {
            for p in 0..COUNTS[col] {
                let idx = phys_index(col, p);
                let c = self.scaled(color, bright);
                Self::max_px(&mut self.fb, idx, c);
            }
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

    /// BRING-UP: verify serpentine wiring, direction, and per-column counts.
    /// Pass 1 walks a single dot through the physical chain (watch it snake). Pass 2
    /// lights each column a distinct colour (check extents/order). Pass 3 walks a dot
    /// along each column from its START end (should always begin at the start, proving
    /// `phys_index` compensates for the reversed odd columns). Loops forever.
    #[cfg(feature = "test-leds")]
    pub async fn test_walk(&mut self, wdt: &mut embassy_rp::watchdog::Watchdog) -> ! {
        use embassy_time::{Duration, Timer};
        const COLS: [RGB8; 5] = [
            RGB8 { r: 80, g: 0, b: 0 },
            RGB8 { r: 0, g: 80, b: 0 },
            RGB8 { r: 0, g: 0, b: 80 },
            RGB8 { r: 70, g: 70, b: 0 },
            RGB8 { r: 0, g: 70, b: 70 },
        ];
        defmt::info!(
            "BRINGUP LED test: pass1 dot walks chain, pass2 columns lit, pass3 per-column start->finish"
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
            // Pass 2 — each column a distinct colour, held ~2 s.
            self.fb = [RGB8::default(); NUM_LEDS];
            for c in 0..5 {
                for p in 0..COUNTS[c] {
                    self.fb[phys_index(c, p)] = COLS[c];
                }
            }
            self.ws.write(&self.fb).await;
            for _ in 0..40 {
                wdt.feed();
                Timer::after(Duration::from_millis(50)).await;
            }
            // Pass 3 — per column, dot from START(pos 0) to finish.
            for c in 0..5 {
                for p in 0..COUNTS[c] {
                    self.fb = [RGB8::default(); NUM_LEDS];
                    self.fb[phys_index(c, p)] = COLS[c];
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
