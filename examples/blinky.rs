#![no_std]
#![no_main]

use embedded_hal_1::delay::DelayNs;
use hal::delay::CycleDelay;
use hal::gpio::{Level, Output, OutputDrive};
use {ch58x_hal as hal, panic_halt as _};

#[qingke_rt::entry]
fn main() -> ! {
    let mut config = hal::Config::default();
    config.clock.use_pll_60mhz().enable_lse();
    let p = hal::init(config);

    let mut delay = CycleDelay;

    // LED PA8
    // let mut led = Output::new(p.PA8, Level::Low, OutputDrive::_5mA);
    let mut led = Output::new(p.PB18, Level::Low, OutputDrive::_5mA);

    loop {
        led.toggle();

        delay.delay_ms(1000);
    }
}
