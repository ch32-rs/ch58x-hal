//! YD-CH582 Board
//! CH582M
#![no_std]
#![no_main]

use ch32v_rt::highcode;
use ch58x_hal as hal;
use embedded_hal_1::delay::DelayUs;
use hal::gpio::{Input, Level, Output, OutputDrive, Pull};
use hal::peripherals;
use hal::rtc::{DateTime, Rtc};
use hal::systick::SysTick;
use hal::uart::UartTx;

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

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    use core::fmt::Write;

    let pa9 = unsafe { peripherals::PA9::steal() };
    let uart1 = unsafe { peripherals::UART1::steal() };
    let mut serial = UartTx::new(uart1, pa9, Default::default()).unwrap();

    let _ = writeln!(&mut serial, "\n\n\n{}", info);

    loop {}
}

#[ch32v_rt::entry]
#[highcode]
fn main() -> ! {
    // LED PA8

    let mut config = hal::Config::default();
    config.clock.use_pll_60mhz();
    let p = hal::init(config);

    let mut delay = SysTick::new(p.SYSTICK);

    // GPIO
    let mut led = Output::new(p.PB4, Level::Low, OutputDrive::Standard);
    let boot_btn = Input::new(p.PB22, Pull::Up);
    let rst_btn = Input::new(p.PB23, Pull::Up);

    let uart = UartTx::new(p.UART1, p.PA9, Default::default()).unwrap();

    unsafe {
        SERIAL.replace(uart);
    }

    let rtc = Rtc::new(p.RTC);

    println!("\nHello World!");
    println!("System Clocks: {}", hal::sysctl::clocks().hclk);
    println!("ChipID: 0x{:02x}", hal::signature::get_chip_id());
    println!("RTC datetime: {}", rtc.now());

    loop {
        led.toggle();
        println!("tick");

        delay.delay_ms(1000);

        println!("button: {} {}", boot_btn.is_low(), rst_btn.is_low());

        if rst_btn.is_low() {
            println!("button: {} {}", boot_btn.is_low(), rst_btn.is_low());
        }
    }
}
