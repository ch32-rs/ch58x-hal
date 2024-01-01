#![no_std]
#![no_main]

use core::fmt::Write;

use display_interface::{DataFormat, WriteOnlyDataCommand};
use display_interface_spi::SPIInterface;
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Line;
use embedded_graphics::text::{Baseline, Text};
use embedded_hal_02::spi::Polarity;
use embedded_hal_1::delay::DelayNs;
use hal::adc::Adc;
use hal::gpio::{Input, Level, Output, OutputDrive, Pull};
use hal::prelude::*;
use hal::rtc::Rtc;
use hal::spi::{BitOrder, Spi};
use hal::uart::UartTx;
use hal::{delay_ms, pac, peripherals, with_safe_access};
use ssd1306::prelude::*;
use ssd1306::rotation::DisplayRotation;
use ssd1306::size::DisplaySize128x32;
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

pub struct DisplaySize128x32Variant1;
impl DisplaySize for DisplaySize128x32Variant1 {
    const WIDTH: u8 = 128;
    const HEIGHT: u8 = 32;
    type Buffer = [u8; Self::WIDTH as usize * Self::HEIGHT as usize / 8];

    fn configure(&self, iface: &mut impl WriteOnlyDataCommand) -> Result<(), display_interface::DisplayError> {
        ssd1306::command::Command::ComPinConfig(true, false).send(iface)?;
        Ok(())
        //ssd1306::command::Command::SegmentRemap(false).send(iface)
        // ssd1306::command::Command::ReverseComDir(false).send(iface)
    }
}

#[qingke_rt::entry]
fn main() -> ! {
    let mut config = hal::Config::default();
    config.clock.use_pll_60mhz();
    let p = hal::init(config);

    // GPIO
    let mut led = Output::new(p.PA8, Level::Low, OutputDrive::_5mA);
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

    let mut spi_config = hal::spi::Config::default();
    spi_config.frequency = 20.MHz();
    spi_config.bit_order = BitOrder::MsbFirst;
    spi_config.clock_polarity = Polarity::IdleLow;
    let mut spi = Spi::new_txonly(p.SPI0, p.PA13, p.PA14, spi_config);

    // pin wiring

    let dc = Output::new(p.PA10, Level::High, OutputDrive::_5mA);
    // active low
    let mut res = Output::new(p.PA4, Level::High, OutputDrive::_5mA);
    // SPI MODE_0, clk idle low, data valid on rising edge
    let cs = Output::new(p.PA5, Level::Low, OutputDrive::_5mA);

    res.set_low();
    delay_ms(100);
    res.set_high();
    delay_ms(100);

    let di = SPIInterface::new(spi, dc, cs);
    let mut display =
        ssd1306::Ssd1306::new(di, DisplaySize128x32Variant1, DisplayRotation::Rotate180).into_buffered_graphics_mode();

    display.init().unwrap();
    let _ = display.set_mirror(true);

    println!("Display initialized");

    for i in 0..128 {
        Line::new(Point::new(0, 0), Point::new(i, 31))
            .into_styled(embedded_graphics::primitives::PrimitiveStyle::with_stroke(
                BinaryColor::On,
                1,
            ))
            .draw(&mut display)
            .unwrap();
        display.flush().unwrap();

        hal::delay_ms(5);
    }
    for i in 0..32 {
        // display.clear(BinaryColor::Off).unwrap();

        Line::new(Point::new(0, 0), Point::new(128, i))
            .into_styled(embedded_graphics::primitives::PrimitiveStyle::with_stroke(
                BinaryColor::On,
                1,
            ))
            .draw(&mut display)
            .unwrap();
        display.flush().unwrap();

        hal::delay_ms(5);
    }

    display.clear(BinaryColor::Off).unwrap();
    display.flush().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    Text::with_baseline("Hello Rust!", Point::new(0, 16), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    display.flush().unwrap();
    hal::delay_ms(1000);

    display.clear(BinaryColor::Off).unwrap();
    display.flush().unwrap();

    let mut buf = heapless::String::<128>::new();

    loop {
        buf.clear();
        display.clear(BinaryColor::Off).unwrap();
        Text::with_baseline("RTC time:", Point::new(0, 5), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        write!(&mut buf, "{}", rtc.now()).unwrap();
        Text::with_baseline(&buf, Point::new(10, 20), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        display.flush().unwrap();
        delay_ms(1000);

        if reset_button.is_low() {
            unsafe {
                hal::reset();
            }
        }
    }
}
