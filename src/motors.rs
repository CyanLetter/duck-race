//! DRV8833 ×2 motor control, shared-reverse scheme (IMPLEMENTATION.md §4).
//!
//! Each motor's IN1 is its own forward-PWM pin (independent speed); all four IN2 pins
//! are tied to ONE shared reverse-PWM pin. In-in fast-decay drive:
//!   forward @duty : IN1=PWM, IN2=0        reverse @duty : IN1=0,  IN2=PWM
//!   coast         : IN1=0,   IN2=0        brake         : IN1=1,  IN2=1
//!
//! PWM wiring: GP2/GP3 = slice1 A/B (lanes 0/1), GP4/GP5 = slice2 A/B (lanes 2/3),
//! GP26 = slice5 A (shared reverse). A/B channels share frequency, independent duty.
//! GP27 (nSLEEP) is slice5 B but used as a plain output — that channel stays unclaimed.
//!
//! On the common cheap DRV8833 breakouts, nSLEEP is silkscreened **EEP** and nFAULT is
//! **ULT**; EEP must be driven high or the driver stays asleep at any duty. One motor
//! uses ONE channel: IN1+IN2 → OUT1+OUT2 (or IN3+IN4 → OUT3+OUT4).

use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{
    PIN_2, PIN_26, PIN_27, PIN_3, PIN_4, PIN_5, PWM_SLICE1, PWM_SLICE2, PWM_SLICE5,
};
use embassy_rp::pwm::{Config as PwmConfig, Pwm};
use embassy_rp::Peri;

use crate::config::{pct_to_compare, LANES, PWM_TOP};

pub struct Motors<'d> {
    fwd01: Pwm<'d>, // channel A = lane 0, channel B = lane 1
    fwd23: Pwm<'d>, // channel A = lane 2, channel B = lane 3
    rev: Pwm<'d>,   // channel A = shared reverse (all IN2)
    cfg01: PwmConfig,
    cfg23: PwmConfig,
    cfg_rev: PwmConfig,
    nsleep: Output<'d>,
}

impl<'d> Motors<'d> {
    pub fn new(
        fwd01: Pwm<'d>,
        fwd23: Pwm<'d>,
        rev: Pwm<'d>,
        cfg: PwmConfig,
        nsleep: Output<'d>,
    ) -> Self {
        Self {
            fwd01,
            fwd23,
            rev,
            cfg01: cfg.clone(),
            cfg23: cfg.clone(),
            cfg_rev: cfg,
            nsleep,
        }
    }

    /// Build the motor set from raw peripherals: GP2/3=slice1, GP4/5=slice2,
    /// GP26=slice5 shared reverse, GP27=nSLEEP. ~20 kHz PWM. Used by `main` and the
    /// bring-up modes.
    #[allow(clippy::too_many_arguments)]
    pub fn build(
        slice1: Peri<'static, PWM_SLICE1>,
        pin2: Peri<'static, PIN_2>,
        pin3: Peri<'static, PIN_3>,
        slice2: Peri<'static, PWM_SLICE2>,
        pin4: Peri<'static, PIN_4>,
        pin5: Peri<'static, PIN_5>,
        slice5: Peri<'static, PWM_SLICE5>,
        pin26: Peri<'static, PIN_26>,
        nsleep: Peri<'static, PIN_27>,
    ) -> Motors<'static> {
        let mut cfg = PwmConfig::default();
        cfg.top = PWM_TOP;
        cfg.divider = 1u8.into();
        let fwd01 = Pwm::new_output_ab(slice1, pin2, pin3, cfg.clone());
        let fwd23 = Pwm::new_output_ab(slice2, pin4, pin5, cfg.clone());
        let rev = Pwm::new_output_a(slice5, pin26, cfg.clone());
        let nsleep = Output::new(nsleep, Level::Low);
        Motors::new(fwd01, fwd23, rev, cfg, nsleep)
    }

    /// nSLEEP high = drivers enabled.
    pub fn enable(&mut self, on: bool) {
        self.nsleep.set_level(if on { Level::High } else { Level::Low });
    }

    fn apply(&mut self) {
        self.fwd01.set_config(&self.cfg01);
        self.fwd23.set_config(&self.cfg23);
        self.rev.set_config(&self.cfg_rev);
    }

    fn set_fwd_compare(&mut self, lane: usize, compare: u16) {
        match lane {
            0 => self.cfg01.compare_a = compare,
            1 => self.cfg01.compare_b = compare,
            2 => self.cfg23.compare_a = compare,
            3 => self.cfg23.compare_b = compare,
            _ => {}
        }
    }

    /// Drive all four lanes forward at the given per-lane duties (%). IN2 held low.
    pub fn race_forward(&mut self, duties_pct: [u8; LANES]) {
        self.cfg_rev.compare_a = 0;
        for l in 0..LANES {
            self.set_fwd_compare(l, pct_to_compare(duties_pct[l]));
        }
        self.apply();
    }

    /// Drive a single lane forward (others left as-is). Used for TUNE test runs.
    pub fn set_lane_forward(&mut self, lane: usize, pct: u8) {
        self.cfg_rev.compare_a = 0;
        self.set_fwd_compare(lane, pct_to_compare(pct));
        self.apply();
    }

    /// Reverse ALL lanes together at one duty (shared line) — homing.
    pub fn reverse_all(&mut self, pct: u8) {
        for l in 0..LANES {
            self.set_fwd_compare(l, 0);
        }
        self.cfg_rev.compare_a = pct_to_compare(pct);
        self.apply();
    }

    /// Brake all lanes (both inputs high) — fast stop at the finish.
    pub fn brake_all(&mut self) {
        for l in 0..LANES {
            self.set_fwd_compare(l, PWM_TOP);
        }
        self.cfg_rev.compare_a = PWM_TOP;
        self.apply();
    }

    /// Coast all lanes (both inputs low).
    pub fn coast_all(&mut self) {
        for l in 0..LANES {
            self.set_fwd_compare(l, 0);
        }
        self.cfg_rev.compare_a = 0;
        self.apply();
    }
}
