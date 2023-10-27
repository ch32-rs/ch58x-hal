#![no_std]
#![no_main]

use core::arch::{asm, global_asm};
use core::fmt::Write;
use core::writeln;

use embedded_hal_1::delay::DelayUs;
use hal::dma::NoDma;
use hal::gpio::{AnyPin, Flex, Input, Level, Output, OutputDrive, Pull};
use hal::interrupt::Interrupt;
use hal::isp::EEPROM_BLOCK_SIZE;
use hal::rtc::{DateTime, Rtc};
use hal::sysctl::Config;
use hal::systick::SysTick;
use hal::uart::UartTx;
use hal::{pac, peripherals, Peripherals};
use {ch58x_hal as hal, panic_halt as _};

static mut SERIAL: Option<UartTx<'static, hal::peripherals::UART1>> = None;

static mut BUTTON: Option<Input<hal::peripherals::PB22>> = None;

#[ch32v_rt::interrupt]
unsafe fn GPIOB() {
    if let Some(button) = BUTTON.as_mut() {
        button.disable_interrupt();

        let serial = unsafe { SERIAL.as_mut().unwrap() };

        writeln!(serial, "in irq handler").unwrap();
        if button.is_low() {
            writeln!(serial, "button pressed").unwrap();
        }
        button.clear_interrupt();
        button.enable_interrupt();
    }
}

#[ch32v_rt::entry]
fn main() -> ! {
    let mut config = hal::Config::default();
    config.clock.use_pll_60mhz().enable_lse();

    let p = hal::init(config);

    let mut delay = SysTick::new(p.SYSTICK);

    // LED PA8
    let mut blue_led = Output::new(p.PA8, Level::Low, OutputDrive::Standard);

    let mut serial = UartTx::new(p.UART1, p.PA9, Default::default()).unwrap();
    unsafe {
        SERIAL = Some(serial);
    }
    //let mut serial = UartTx::new(p.UART3, p.PA5, Default::default()).unwrap();
    //let mut serial = UartTx::new(p.UART0, p.PA14, Default::default()).unwrap();

    //let mut serial = UartTx::new(p.UART0, p.PB7, Default::default()).unwrap();

    let mut button = Input::new(p.PB22, Pull::Up);
    // let mut reset_button = Input::new(p.PB23, Pull::Up);

    button.set_trigger(hal::gpio::InterruptTrigger::FallingEdge);
    button.enable_interrupt();
    unsafe {
        BUTTON = Some(button);
    }
    unsafe { hal::interrupt::GPIOB::enable() };

    let mut rtc = Rtc {};

    let serial = unsafe { SERIAL.as_mut().unwrap() };

    writeln!(serial, "\n\n\nHello World!").unwrap();
    writeln!(serial, "Clocks: {}", hal::sysctl::clocks().hclk).unwrap();
    writeln!(serial, "ChipID: {:02x}", hal::signature::get_chip_id());
    let now = rtc.now();
    writeln!(serial, "Boot time: {} weekday={}", now, now.isoweekday()).unwrap();

    loop {
        blue_led.toggle();

        // writeln!(uart, "day {:?}", rtc.counter_day()).unwrap();
        // writeln!(uart, "2s {:?}", rtc.counter_2s()).unwrap();

        //  writeln!(uart, "tick! {}", SysTick::now()).unwrap();
        delay.delay_ms(1000);

        let now = rtc.now();
        writeln!(
            serial,
            "{}: tick",
            now,
            // now.isoweekday(),
        )
        .unwrap();
        // serial.blocking_flush();
        //writeln!(serial, "Current time: {} weekday={}", now, now.isoweekday()).unwrap();
        //writeln!(serial, "button: {} {}", ).unwrap();
    }
}
