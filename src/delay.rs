//! Simple busy-loop delay provider

use qingke::riscv;
use qingke_rt::highcode;

use crate::sysctl::clocks;

pub struct CycleDelay;

impl embedded_hal_1::delay::DelayNs for CycleDelay {
    #[highcode]
    fn delay_us(&mut self, us: u32) {
        let cycles = us as u64 * clocks().hclk.to_Hz() as u64 / 1_500_000;
        riscv::asm::delay(cycles as u32);
    }

    #[highcode]
    fn delay_ns(&mut self, ns: u32) {
        let cycles = ns as u64 * clocks().hclk.to_Hz() as u64 / 1_500_000_000;
        riscv::asm::delay(cycles as u32);
    }

    #[highcode]
    fn delay_ms(&mut self, mut ms: u32) {
        let cycles = 1000 * clocks().hclk.to_Hz() as u64 / 1_500_000;

        while ms > 0 {
            riscv::asm::delay(cycles as u32);
            ms -= 1;
        }
    }
}
