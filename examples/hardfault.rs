#![no_std]
#![no_main]

use core::arch::global_asm;
use core::writeln;
use core::{arch::asm, fmt::Write};

use ch58x_hal as hal;
use embedded_hal_alpha::delay::DelayUs;
use hal::gpio::{AnyPin, Input, Level, Output, OutputDrive, Pull};
use hal::isp::EEPROM_BLOCK_SIZE;
use hal::systick::SysTick;
use hal::{
    pac,
    rtc::{DateTime, Rtc},
    serial::Uart,
    sysctl::Config,
};
use hal::{peripherals, Peripherals};
use panic_halt as _;

global_asm!(
    r#"
    .section .trap, "ax"
    .global RTC
RTC:
    addi sp, sp, -4
    sw ra, 0(sp)
    jal _rust_RTC
    lw ra, 0(sp)
    addi sp, sp, 4
    mret
"#
);

#[allow(non_snake_case)]
#[export_name = "_rust_RTC"]
extern "C" fn RTC_IRQHandler() {
    let mut rtc = Rtc;
    let mut uart = Uart {};

    writeln!(uart, "Entering IRQ...");

    rtc.ack_timing();

    let now = rtc.now();
    writeln!(uart, "Current time: {} weekday={}", now, now.isoweekday()).unwrap();
    //  writeln!(uart, "mepc: {:08x}", riscv::register::mepc::read()).unwrap();
}

#[allow(non_snake_case)]
#[export_name = "HardFault"]
extern "C" fn HardFault_IRQHandler() {
    let pa8 = unsafe { hal::peripherals::PA8::steal() };
    let mut led = Output::new(pa8, Level::Low, OutputDrive::High);

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

#[link_section = ".sbss"]
static mut BUF: [u8; 1024] = [0; 1024];

// #[riscv_rt::entry]
#[allow(non_snake_case)]
#[export_name = "main"]
extern "C" fn main() -> ! {
    // LED PA8
    // hal::sysctl::Config::pll_60mhz().freeze();
    hal::sysctl::Config::pll_60mhz().use_lse().freeze();

    let p = Peripherals::take();

    let mut delay = SysTick::new(p.SYSTICK);

    unsafe {
        // prepare PA9, for uart tx
        p.GPIO.pa_dir.modify(|_, w| w.pa_dir().bits((1 << 9)));
    }

    let mut pa8 = Output::new(p.PA8, Level::Low, OutputDrive::Low);

    let mut download_button = Input::new(p.PB22, Pull::Up);
    let mut reset_button = Input::new(p.PB23, Pull::Up);

    let mut uart = Uart::new(Default::default());

    writeln!(uart, "\n\n\nHello World!").unwrap();
    writeln!(uart, "Clocks: {}", hal::sysctl::clocks().hclk).unwrap();
    writeln!(uart, "ChipID: {:02x}", hal::signature::get_chip_id());

    //let boot_info = hal::isp::get_boot_info();
    //writeln!(uart, "boot_info: {:02x?}", boot_info).unwrap();

    //uart.flush();
    //let ret = unsafe { hal::isp::eeprom_read(0x0, &mut BUF[..500]) };
    // writeln!(uart, "ret {}", ret);
    // unsafe { writeln!(uart, "read flash: {:02x?}", &BUF[..500]).unwrap() };

    /// erase 1 block(4k)
    uart.flush();
    // let ret = hal::isp::eeprom_erase(0x0, 256);
    // writeln!(uart, "erase ret {}", ret);

    let mut rtc = Rtc;

    let pfic = unsafe { &*pac::PFIC::PTR };

    rtc.enable_timing(hal::rtc::TimingMode::_2S);
    unsafe { pfic.ienr1.write(|w| w.bits(1 << 28)) }; // enable rtc irq

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
            delay.delay_ms(100);

            while download_button.is_low() {}

            if reset_button.is_low() {
                writeln!(
                    uart,
                    "button: {} {}",
                    download_button.is_low(),
                    reset_button.is_low()
                )
                .unwrap();
            }
            unsafe { asm!("j -222224") };
        }
    }
}
