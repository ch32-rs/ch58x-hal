#![no_std]
#![no_main]

use core::arch::{asm, global_asm};
use core::fmt::Write;
use core::writeln;

use embedded_hal_alpha::delay::DelayUs;
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
    let mut blue_led = Output::new(p.PA8, Level::Low, OutputDrive::Low);

    let mut serial = UartTx::new(p.UART1, p.PA9, Default::default()).unwrap();
    //let mut serial = UartTx::new(p.UART3, p.PA5, Default::default()).unwrap();
    //let mut serial = UartTx::new(p.UART0, p.PA14, Default::default()).unwrap();

    //let mut serial = UartTx::new(p.UART0, p.PB7, Default::default()).unwrap();

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
    let adc = unsafe { &*pac::ADC::PTR };
    adc.tem_sensor.modify(|_, w| w.tem_sen_pwr_on().set_bit());
    adc.channel.modify(|_, w| w.ch_inx().variant(15));

    // 00, 01, 02, 03
    adc.cfg.modify(|_, w| {
        w.power_on()
            .set_bit()
            .diff_en()
            .set_bit() // must for temp
            .clk_div()
            .variant(0b00)
            .buf_en()
            .clear_bit()
            .pga_gain()
            .variant(0b11)
    });
    // start adc convert

    const ROM_CFG_TMP_25C: *const u32 = 0x7F014 as *const u32;

    let c25 = unsafe { core::ptr::read_volatile(ROM_CFG_TMP_25C) };

    writeln!(serial, "c25: 0x{:08x}", c25).unwrap();

    loop {
        blue_led.toggle();

        adc.convert.modify(|_, w| w.start().set_bit());
        while adc.convert.read().start().bit_is_set() {} // wait for convert

        let data = adc.data.read().data().bits();
        // writeln!(serial, "adc data: {}", data,).unwrap();
        let temp_celesius = hal::adc::adc_to_temperature_milli_celsius(data);
        writeln!(serial, "temp: {}.{}C", temp_celesius / 1000, temp_celesius % 1000 / 100).unwrap();

        // writeln!(uart, "day {:?}", rtc.counter_day()).unwrap();
        // writeln!(uart, "2s {:?}", rtc.counter_2s()).unwrap();

        //  writeln!(uart, "tick! {}", SysTick::now()).unwrap();
        delay.delay_ms(500);

        let now = rtc.now();
        writeln!(
            serial,
            "{}:  button: download={} reset={}",
            now,
            // now.isoweekday(),
            download_button.is_low(),
            reset_button.is_low()
        )
        .unwrap();
        // serial.blocking_flush();
        //writeln!(serial, "Current time: {} weekday={}", now, now.isoweekday()).unwrap();
        //writeln!(serial, "button: {} {}", ).unwrap();
    }
}
