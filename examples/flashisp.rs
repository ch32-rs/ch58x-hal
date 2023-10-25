#![no_std]
#![no_main]

use ch32v_rt::highcode;
use embedded_hal_1::delay::DelayUs;
use hal::gpio::{Input, Level, Output, OutputDrive, Pull};
// use hal::interrupt::Interrupt;
use hal::rtc::{DateTime, Rtc};
use hal::systick::SysTick;
use hal::uart::UartTx;
use hal::{peripherals, with_safe_access};
use {ch58x_hal as hal, panic_halt as _};

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

#[ch32v_rt::entry]
#[highcode]
fn main() -> ! {
    // LED PA8

    let mut config = hal::Config::default();
    config.clock.use_pll_60mhz();
    let p = hal::init(config);

    let mut delay = SysTick::new(p.SYSTICK);

    // GPIO
    let mut led = Output::new(p.PA8, Level::Low, OutputDrive::Standard);
    let download_button = Input::new(p.PB22, Pull::Up);
    let reset_button = Input::new(p.PB23, Pull::Up);

    let uart = UartTx::new(p.UART1, p.PA9, Default::default()).unwrap();
    unsafe {
        SERIAL.replace(uart);
    }

    let rtc = Rtc::new(p.RTC);

    println!("\nHello World!");
    println!("System Clocks: {}", hal::sysctl::clocks().hclk);
    println!("ChipID: 0x{:02x}", hal::signature::get_chip_id());
    println!("RTC datetime: {}", rtc.now());
    let uid = hal::isp::get_unique_id();
    println!("Chip UID: {:02x?}", uid);

    let mac_address = hal::isp::get_mac_address();
    println!("MAC Address: {:02x?}", mac_address);

    let boot_info = hal::isp::get_boot_info();
    println!("Boot Info: {:02x?}", boot_info);

    let s = hal::isp::get_user_option();
    println!("User Option: {:02x?}", s);

    if s & 0x08 != 0 {
        println!("-- Reset Enable");
    } else {
        println!("-- Reset Disable");
    }
    let s = (!(s << 24)) & 0xFF000000;

    loop {
        led.toggle();
        println!("tick");

        delay.delay_ms(1000);

        /*
        while download_button.is_low() {}
        if reset_button.is_low() {
            println!("button: {} {}", download_button.is_low(), reset_button.is_low());
        }
        */
    }
}
