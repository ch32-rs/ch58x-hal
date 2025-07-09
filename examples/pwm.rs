#![no_std]
#![no_main]

use core::arch::{asm, global_asm};
use core::fmt::Write;
use core::writeln;

use ch58x_hal::pwm::{self, Pwm, PwmPin};
use embedded_hal_1::delay::DelayNs;
use hal::gpio::{AnyPin, Input, Level, Output, OutputDrive, Pull};
use hal::sysctl::Config;
use hal::uart::UartTx;
use hal::{pac, peripherals, Peripherals};
use {ch58x_hal as hal, panic_halt as _};

#[qingke_rt::entry]
fn main() -> ! {
    let mut config = hal::Config::default();
    config.clock.use_pll_60mhz().enable_lse();

    let p = hal::init(config);

    // LED PB4
    let mut led = Output::new(p.PB4, Level::Low, OutputDrive::_5mA);

    let mut serial = UartTx::new(p.UART1, p.PA9, Default::default()).unwrap();

    let mut pwm_config = pwm::Config {
        clock_div: 4,
        cycle_sel: pwm::Cycle::Cycle64,
    };

    let mut pwm = Pwm::new(p.PWMX, pwm_config);

    let mut pwm_pin = led;

    pwm.set_polarity(true, &mut pwm_pin);
    pwm.set_duty(255, &mut pwm_pin);
    pwm.enable(&mut pwm_pin);

    loop {
        for duty in (0..255).rev() {
            pwm.set_duty(duty, &mut pwm_pin);
            hal::delay_ms(10);
        }
    }
}
