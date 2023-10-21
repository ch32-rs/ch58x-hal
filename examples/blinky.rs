#![no_std]
#![no_main]

use core::arch::{asm, global_asm};
use core::fmt::Write;
use core::writeln;

use embedded_hal_1::delay::DelayUs;
use hal::gpio::{AnyPin, Input, Level, Output, OutputDrive, Pull};
use hal::interrupt::Interrupt;
use hal::isp::EEPROM_BLOCK_SIZE;
use hal::rtc::{DateTime, Rtc};
use hal::sysctl::Config;
use hal::systick::SysTick;
use hal::uart::UartTx;
use hal::{pac, peripherals, Peripherals};
use {ch58x_hal as hal, panic_halt as _};

#[ch32v_rt::entry]
fn main() -> ! {
    // LED PA8
    // hal::sysctl::Config::pll_60mhz().freeze();
    ///hal::sysctl::Config::pll_60mhz().use_lse().freeze();
    let mut config = hal::Config::default();
    config.clock.use_pll_60mhz().enable_lse();
    config.enable_dcdc = true;
    config.low_power = true;
    config.clock.enable_lse();

    //let p = Peripherals::take();
    let p = hal::init(config);

    let mut delay = SysTick::new(p.SYSTICK);

    let mut pa8 = Output::new(p.PA8, Level::Low, OutputDrive::Low);

    let mut download_button = Input::new(p.PB22, Pull::Up);
    let mut reset_button = Input::new(p.PB23, Pull::Up);

    let mut uart = UartTx::new(p.UART1, p.PA9, Default::default()).unwrap();

    writeln!(uart, "\n\n\nHello World!").unwrap();
    writeln!(uart, "Clocks: {}", hal::sysctl::clocks().hclk).unwrap();
    writeln!(uart, "ChipID: {:02x}", hal::signature::get_chip_id());

    let mut rtc = Rtc;

    //unsafe {
    //    hal::interrupt::RTC::enable();
    //}
    //rtc.enable_timing(hal::rtc::TimingMode::_2S);
    //F    let pfic = unsafe { &*pac::PFIC::PTR };
    //    unsafe { pfic.ienr1.write(|w| w.bits(1 << 28)) }; // enable rtc irq

    /*
    rtc.set_datatime(DateTime {
        year: 2023,
        month: 10,
        day: 12,
        hour: 18,
        minute: 45,
        second: 0,
    });
    */

    // let buf = hal::isp::read_eeprom(0x0, 500);
    // writeln!(uart, "read flash: {:02x?}", buf).unwrap();

    // ISP functions

    loop {
        unsafe {
            pa8.toggle();

            // writeln!(uart, "day {:?}", rtc.counter_day()).unwrap();
            // writeln!(uart, "2s {:?}", rtc.counter_2s()).unwrap();

            //  writeln!(uart, "tick! {}", SysTick::now()).unwrap();
            //delay.delay_ms(1000);
            hal::delay_ms(1000);

            writeln!(uart, "toggle led").unwrap();

            if reset_button.is_low() {
                writeln!(uart, "button: {} {}", download_button.is_low(), reset_button.is_low()).unwrap();
            }
        }
    }
}
