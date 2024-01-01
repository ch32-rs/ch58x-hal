#![no_std]
#![no_main]

use embedded_hal_1::delay::DelayNs;
use hal::gpio::{Input, Level, Output, OutputDrive, Pull};
use hal::peripherals;
use hal::uart::UartTx;
use {ch58x_hal as hal, panic_halt as _};
// use hal::interrupt::Interrupt;
// use hal::rtc::{DateTime, Rtc};

static mut SERIAL: Option<UartTx<peripherals::UART1>> = None;

macro_rules! println {
    ($($arg:tt)*) => {
        unsafe {
            use core::fmt::Write;
            use core::writeln;

            if let Some(uart) = SERIAL.as_mut() {
                writeln!(uart, $($arg)*).unwrap();
            }
        }
    }
}

#[qingke_rt::entry]
fn main() -> ! {
    // LED PA8

    let mut config = hal::Config::default();
    config.clock.use_pll_80mhz();
    let p = hal::init(config);

    let mut led = Output::new(p.PA8, Level::Low, OutputDrive::_5mA);

    let download_button = Input::new(p.PB22, Pull::Up);
    let reset_button = Input::new(p.PB23, Pull::Up);

    let uart = UartTx::new(p.UART1, p.PA9, Default::default()).unwrap();
    unsafe {
        SERIAL.replace(uart);
    }

    println!("\nHello World!");
    println!("Clocks: {}", hal::sysctl::clocks().hclk);
    println!("ChipID: {:02x}", hal::signature::get_chip_id());

    loop {
        println!("tick");
        led.toggle();

        hal::delay_ms(1000);

        while download_button.is_low() {}

        if reset_button.is_low() {
            println!("button: {} {}", download_button.is_low(), reset_button.is_low());
        }
    }
}
