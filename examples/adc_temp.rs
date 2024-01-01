#![no_std]
#![no_main]

use core::arch::{asm, global_asm};
use core::fmt::Write;
use core::writeln;

use embedded_hal_1::delay::DelayNs;
use hal::adc::{adc_to_temperature_celsius, Adc};
use hal::dma::NoDma;
use hal::gpio::{AnyPin, Input, Level, Output, OutputDrive, Pull};
use hal::interrupt::Interrupt;
use hal::isp::EEPROM_BLOCK_SIZE;
use hal::rtc::{DateTime, Rtc};
use hal::sysctl::Config;
use hal::uart::UartTx;
use hal::{pac, peripherals, Peripherals};
use {ch58x_hal as hal, panic_halt as _};

#[qingke_rt::entry]
fn main() -> ! {
    let mut config = hal::Config::default();
    config.clock.use_pll_60mhz().enable_lse();
    let p = hal::init(config);

    // LED PA8
    let mut blue_led = Output::new(p.PA8, Level::Low, OutputDrive::_5mA);

    let mut serial = UartTx::new(p.UART1, p.PA9, Default::default()).unwrap();

    let mut download_button = Input::new(p.PB22, Pull::Up);
    let mut reset_button = Input::new(p.PB23, Pull::Up);
    let mut rtc = Rtc {};

    serial.blocking_flush().unwrap();
    //      rtc.set_datatime(DateTime {
    //        year: 2023,
    //        month: 10,
    //        day: 16,
    //        hour: 15,
    //        minute: 42,
    //        second: 10,
    //    });

    writeln!(serial, "\n\n\nHello World!").unwrap();
    writeln!(serial, "Clocks: {}", hal::sysctl::clocks().hclk).unwrap();
    writeln!(serial, "ChipID: {:02x}", hal::signature::get_chip_id()).unwrap();
    let now = rtc.now();
    writeln!(serial, "Boot time: {} weekday={}", now, now.isoweekday()).unwrap();

    // ADC part
    let adc_config = hal::adc::Config::for_temperature();
    let mut adc = Adc::new(p.ADC, adc_config);

    let mut temp_sensor = adc.enable_temperature();

    loop {
        blue_led.toggle();

        let now = rtc.now();

        let raw_temp = adc.read(&mut temp_sensor);
        writeln!(serial, "ADC raw data: {}", raw_temp).unwrap();
        let temp = adc_to_temperature_celsius(raw_temp);
        writeln!(serial, "sensor temp: {}C", temp).unwrap();

        let vi = adc.read_as_millivolts(&mut temp_sensor);
        writeln!(serial, "ADC voltage: {}mV", vi).unwrap();

        writeln!(
            serial,
            "{}: download={} reset={}",
            now,
            // now.isoweekday(),
            download_button.is_low(),
            reset_button.is_low()
        )
        .unwrap();

        hal::delay_ms(1000);
    }
}
