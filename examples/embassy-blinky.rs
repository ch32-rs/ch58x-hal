#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch58x_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Instant, Timer};
use hal::gpio::{AnyPin, Input, Level, Output, OutputDrive, Pin, Pull};
use hal::peripherals;
use hal::prelude::*;
use hal::rtc::Rtc;
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

#[embassy_executor::task]
async fn blink(pin: AnyPin) {
    let mut led = Output::new(pin, Level::Low, OutputDrive::_5mA);

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(150)).await;
        led.set_low();
        Timer::after(Duration::from_millis(150)).await;
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    let mut config = hal::Config::default();
    config.clock.use_pll_60mhz();
    let p = hal::init(config);
    hal::embassy::init();

    let uart = UartTx::new(p.UART1, p.PA9, Default::default()).unwrap();
    unsafe {
        SERIAL.replace(uart);
    }

    // GPIO
    spawner.spawn(blink(p.PA8.degrade())).unwrap();

    let reset_button = Input::new(p.PB23, Pull::Up);

    let rtc = Rtc::new(p.RTC);

    println!("\n\nHello World from ch58x-hal!");
    println!(
        r#"
    ______          __
   / ____/___ ___  / /_  ____ _____________  __
  / __/ / __ `__ \/ __ \/ __ `/ ___/ ___/ / / /
 / /___/ / / / / / /_/ / /_/ (__  |__  ) /_/ /
/_____/_/ /_/ /_/_.___/\__,_/____/____/\__, /
                                      /____/   on CH582F"#
    );
    println!("System Clocks: {}", hal::sysctl::clocks().hclk);
    println!("ChipID: 0x{:02x}", hal::signature::get_chip_id());
    println!("RTC datetime: {}", rtc.now());

    loop {
        //led.toggle();
        println!("inst => {:?}", Instant::now());
        // Delay.delay_ms(1000_u32); // blocking delay
        Timer::after(Duration::from_millis(1000)).await;

        if reset_button.is_low() {
            unsafe {
                hal::reset();
            }
        }
    }
}
