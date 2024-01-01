#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch58x_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_nb::serial::Write;
use hal::gpio::{AnyPin, Input, Level, Output, OutputDrive, Pin, Pull};
use hal::rtc::Rtc;
use hal::uart::UartTx;
use hal::{peripherals, println};

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

#[embassy_executor::task]
async fn reset_if_requested(pin: AnyPin) {
    let mut reset_btn = Input::new(pin, Pull::Up);

    reset_btn.wait_for_rising_edge().await;

    unsafe {
        // hal::reset();
        hal::soft_reset();
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    let mut config = hal::Config::default();
    config.clock.use_pll_60mhz().enable_lse();
    let p = hal::init(config);
    hal::embassy::init();

    let uart = UartTx::new(p.UART1, p.PA9, Default::default()).unwrap();
    unsafe {
        hal::set_default_serial(uart);
    }

    // GPIO
    spawner.spawn(blink(p.PB4.degrade())).unwrap();

    spawner.spawn(reset_if_requested(p.PB23.degrade())).unwrap();

    let mut boot_btn = Input::new(p.PB22, Pull::Up);

    let rtc = Rtc::new(p.RTC);

    println!("\n\n\n");
    println!("Hello World from ch58x-hal!");
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

    println!("Awaiting boot button press... (press reset to exit)");

    loop {
        boot_btn.wait_for_rising_edge().await;

        println!("Boot pressed!!");
        println!("inst => {}", rtc.now());

        // Delay.delay_ms(1000_u32); // blocking delay
        // Timer::after(Duration::from_millis(10000)).await;
    }
}
