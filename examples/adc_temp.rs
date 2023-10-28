#![no_std]
#![no_main]

use core::arch::{asm, global_asm};
use core::fmt::Write;
use core::writeln;

use embedded_hal_1::delay::DelayUs;
use hal::adc::{adc_to_temperature_celsius, Adc};
use hal::dma::NoDma;
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
    let mut config = hal::Config::default();
    config.clock.use_pll_60mhz().enable_lse();

    let p = hal::init(config);

    let mut delay = SysTick::new(p.SYSTICK);

    // LED PA8
    let mut blue_led = Output::new(p.PA8, Level::Low, OutputDrive::_5mA);

    let mut serial = UartTx::new(p.UART1, p.PA9, Default::default()).unwrap();

    let mut download_button = Input::new(p.PB22, Pull::Up);
    let mut reset_button = Input::new(p.PB23, Pull::Up);
    let mut rtc = Rtc {};

    serial.blocking_flush();
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
    writeln!(serial, "ChipID: {:02x}", hal::signature::get_chip_id());
    let now = rtc.now();
    writeln!(serial, "Boot time: {} weekday={}", now, now.isoweekday()).unwrap();

    let marchid = riscv::register::marchid::read().unwrap();
    writeln!(serial, "marchid: 0x{:08x?}", marchid.bits());
    let mias = riscv::register::misa::read().unwrap();
    writeln!(serial, "mias: 0x{:08x?}", mias.bits());

    // ADC part
    let mut adc_config = hal::adc::Config::for_temperature();
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

        /*

        // start adc convert
        let data = adc.read(&mut vbat_channel);
        writeln!(serial, "adc raw data: {}", data).unwrap();
        // avoid using soft-fp, about 20k flash increase
        // let vref = 1.05;
        // let vi = ((data as f32) / 4096.0 + 0.5) * vref;
        let vref = 1050;
        let vi = ((data as u32) * vref) / 4096 + 1050 / 2;
        writeln!(serial, "Vbat voltage: {}mV", vi).unwrap();

        let vi = adc.read_as_millivolts(&mut vbat_channel);
        writeln!(serial, "Vbat voltage: {}mV", vi).unwrap();

        //let raw_temp = adc.read(&mut temp_sensor);
        //writeln!(serial, "raw_temp: {}", raw_temp);
        //let temp = adc_to_temperature_celsius(raw_temp);
        //writeln!(serial, "sensor temp: {}C", temp).unwrap();
        */

        writeln!(
            serial,
            "{}: download={} reset={}",
            now,
            // now.isoweekday(),
            download_button.is_low(),
            reset_button.is_low()
        )
        .unwrap();

        delay.delay_ms(1000);
    }
}
