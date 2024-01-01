#![no_std]
#![no_main]

use core::arch::{asm, global_asm};
use core::fmt::Write;
use core::writeln;

use embedded_hal_1::delay::DelayNs;
use hal::gpio::{AnyPin, Input, Level, Output, OutputDrive, Pull};
use hal::isp::EEPROM_BLOCK_SIZE;
use hal::rtc::{DateTime, Rtc};
use hal::sysctl::Config;
use hal::uart::UartTx;
use hal::{pac, peripherals, Peripherals};
use qingke::riscv;
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

extern "C" fn RTC() {
    let mut rtc = Rtc;

    rtc.ack_timing();

    let now = rtc.now();
    println!("Current time: {} weekday={}", now, now.isoweekday());
    //  writeln!(uart, "mepc: {:08x}", riscv::register::mepc::read()).unwrap();
}
extern "C" fn HardFault() {
    let pa8 = unsafe { hal::peripherals::PA8::steal() };
    let mut led = Output::new(pa8, Level::Low, OutputDrive::_20mA);

    println!("in hardfault");
    let mcause = riscv::register::mcause::read();
    println!("mcause: {:?}", mcause);

    let short = hal::sysctl::clocks().hclk.to_Hz() / 256;
    let long = hal::sysctl::clocks().hclk.to_Hz() / 32;

    // blink pattern .. _
    loop {
        led.set_low(); // active low
        unsafe { riscv::asm::delay(short) };
        led.set_high();
        unsafe { riscv::asm::delay(short) };
        led.set_low();
        unsafe { riscv::asm::delay(short) };
        led.set_high();
        unsafe { riscv::asm::delay(short) };
        led.set_low();
        unsafe { riscv::asm::delay(long) };
        led.set_high();
        unsafe { riscv::asm::delay(long) };
    }
}

#[qingke_rt::entry]
fn main() -> ! {
    let mut config = hal::Config::default();
    config.clock.use_pll_60mhz();
    let p = hal::init(config);

    let mut pa8 = Output::new(p.PA8, Level::Low, OutputDrive::_5mA);

    let mut download_button = Input::new(p.PB22, Pull::Up);
    let mut reset_button = Input::new(p.PB23, Pull::Up);

    let uart = UartTx::new(p.UART1, p.PA9, Default::default()).unwrap();
    unsafe { SERIAL.replace(uart) };

    println!("\nMCU init ok!");

    let mut rtc = Rtc;

    let pfic = unsafe { &*pac::PFIC::PTR };

    rtc.enable_timing(hal::rtc::TimingMode::_2S);
    unsafe {
        qingke::pfic::enable_interrupt(pac::Interrupt::RTC as u8);
    }
    loop {
        unsafe {
            pa8.toggle();

            hal::delay_ms(100);

            while download_button.is_low() {}

            if reset_button.is_low() {
                println!("button: {} {}", download_button.is_low(), reset_button.is_low());
            }
            unsafe { asm!("j -222224") }; // trigger hardfault
        }
    }
}
