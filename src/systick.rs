use crate::pac;
use crate::peripheral::{Peripheral, PeripheralRef};
use crate::peripherals::SYSTICK;

pub struct SysTick<'d> {
    _inner: PeripheralRef<'d, SYSTICK>,
    ticks_per_second: u64,
}

impl<'d> SysTick<'d> {
    pub fn new(p: impl Peripheral<P = SYSTICK> + 'd) -> Self {
        crate::into_ref!(p);

        let systick = unsafe { &*pac::SYSTICK::PTR };

        let ticks_per_second = crate::sysctl::clocks().hclk.to_Hz() as u64 / 8;

        systick.ctlr.modify(|_, w| {
            w.init()
                .set_bit()
                .mode()
                .upcount()
                .stre()
                .clear_bit() // no reload
                .stclk()
                .hclk_div8()
                //.stie().set_bit() // disable interrupt
                .ste()
                .set_bit()
        });

        Self {
            _inner: p,
            ticks_per_second,
        }
    }

    pub fn now() -> u64 {
        let systick = unsafe { &*pac::SYSTICK::PTR };

        systick.cnt.read().bits()
    }
}

impl<'d> embedded_hal_1::delay::DelayNs for SysTick<'d> {
    fn delay_us(&mut self, us: u32) {
        let us = self.ticks_per_second * (us as u64) / 1_000_000;

        let target = Self::now() + us;
        while Self::now() < target {}
    }

    fn delay_ms(&mut self, ms: u32) {
        let ms = self.ticks_per_second * (ms as u64) / 1_000;

        let target = Self::now() + ms;
        while Self::now() < target {}
    }
}
