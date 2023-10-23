// interrupt handling for SysTick
#![no_std]
#![no_main]

use core::arch::global_asm;

use embassy_time::{Delay, Instant};
use hal::gpio::{Input, Level, Output, OutputDrive, Pull};
use hal::interrupt::{self, Interrupt};
// use hal::interrupt::Interrupt;
use hal::pac;
use hal::prelude::*;
use hal::rtc::{DateTime, Rtc};
use hal::systick::SysTick;
use hal::uart::UartTx;
use hal::{peripherals, with_safe_access};
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

#[ch32v_rt::entry]
fn main() -> ! {
    // LED PA8

    let mut config = hal::Config::default();
    config.clock.use_pll_60mhz();
    let p = hal::init(config);

    // GPIO
    let mut led = Output::new(p.PA8, Level::Low, OutputDrive::Low);
    let download_button = Input::new(p.PB22, Pull::Up);
    let reset_button = Input::new(p.PB23, Pull::Up);

    let uart = UartTx::new(p.UART1, p.PA9, Default::default()).unwrap();
    unsafe {
        SERIAL.replace(uart);
    }

    let mut rtc = Rtc::new(p.RTC);

    println!("\nHello World!");
    println!("System Clocks: {}", hal::sysctl::clocks().hclk);
    println!("ChipID: 0x{:02x}", hal::signature::get_chip_id());
    println!("RTC datetime: {}", rtc.now());

    let systick = unsafe { &*pac::SYSTICK::PTR };
    let ticks_per_second = hal::sysctl::clocks().hclk.to_Hz() as u64 / 8;

    //hal::embassy::init();

    // Avoid initial interrupt, this step is undocumented in datasheet
    systick.cmp.write(|w| unsafe { w.bits(u64::MAX) });
    systick.ctlr.modify(|_, w| {
        w.init()
            .set_bit()
            .mode()
            .upcount()
            .stre()
            .clear_bit() // no reload
            .stclk()
            .hclk_div8()
            .stie()
            .set_bit()
            .ste()
            .set_bit()
    });
    unsafe {
        interrupt::SysTick::enable();
    }

    let target: u64 = 60_000_000 / 8 * 10; // 10 sec after start
    systick.cmp.write(|w| unsafe { w.bits(target) });
    // systick.ctlr.modify(|_, w| w.stie().set_bit()); // enable interrupt

    loop {
        led.toggle();
        println!("tick");

        println!("counter => {}", systick.cnt.read().bits());

        // unsafe { riscv::asm::wfi() };
        // println!("after wfi");

        hal::delay_ms(1000);

        if reset_button.is_low() {
            println!("reset button pressed");
            unsafe { hal::reset() };
        }
    }
}

core::arch::global_asm!(
    r#"
    .section .trap, "ax"
    .global SysTick
SysTick:
    addi sp, sp, -4
    sw ra, 0(sp)
    jal _rust_SysTick
    lw ra, 0(sp)
    addi sp, sp, 4
    mret
"#
);

#[allow(non_snake_case)]
#[export_name = "_rust_SysTick"]
extern "C" fn SysTick_IRQHandler() {
    // FIXME: the usage of SWIE is unknown
    let systick = unsafe { &*pac::SYSTICK::PTR };
    // systick.ctlr.modify(|_, w| w.stie().clear_bit()); // disable interrupt
    systick.sr.write(|w| w.cntif().clear_bit()); // clear IF

    let cnt = systick.cnt.read().bits();
    println!("in IRQ...");
    println!("IRQ CNT {}", cnt);
    println!("IRQ CMP {}", systick.cmp.read().bits());

    let period: u64 = 60_000_000 / 8 * 5; // 5s

    let target = cnt + period;
    systick.cmp.write(|w| unsafe { w.bits(target) });

    // systick.ctlr.modify(|_, w| w.stie().set_bit()); // re-enable interrupt
}
