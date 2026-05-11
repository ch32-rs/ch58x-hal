#![no_std]
#![recursion_limit = "1024"]
use core::ptr;

pub use ch58x::ch58x as pac;
use qingke::riscv;

pub use self::peripheral::{Peripheral, PeripheralRef};
pub use self::peripherals::Peripherals;

pub mod adc;
pub mod dma;
pub mod gpio;
pub mod i2c;
// pub mod lcd;
#[cfg(feature = "ble")]
pub mod ble;
pub mod delay;
pub mod rtc;
pub mod signature;
pub mod spi;
pub mod sysctl;
#[cfg(not(feature = "embassy"))]
pub mod systick;
pub mod timer;
pub mod uart;

pub mod interrupt;
pub mod isp;
// pub mod rt;

pub mod peripherals;

mod peripheral;
pub mod prelude;

#[cfg(feature = "embassy")]
pub mod embassy;

/// Bits per second
pub type BitsPerSecond = fugit::HertzU32;

/// Extension trait that adds convenience methods to the `u32` type
pub trait U32Ext {
    /// Wrap in `Bps`
    fn bps(self) -> BitsPerSecond;
}

impl U32Ext for u32 {
    fn bps(self) -> BitsPerSecond {
        BitsPerSecond::from_raw(self)
    }
}

static mut IRQ_STA: usize = 0;

pub fn with_safe_access<F, R>(f: F) -> R
where
    F: FnOnce() -> R,
{
    use qingke::register::gintenr;

    const REG_SAFE_ACCESS_SIG: *mut u8 = 0x40001040 as *mut u8;
    const SAFE_ACCESS_SIG1: u8 = 0x57;
    const SAFE_ACCESS_SIG2: u8 = 0xA8;

    unsafe {
        if gintenr::read() & 0x08 != 0 {
            IRQ_STA = gintenr::read();
            gintenr::write(IRQ_STA & (!0x08));
        }
        riscv::asm::nop();
        riscv::asm::nop();

        ptr::write_volatile(REG_SAFE_ACCESS_SIG, SAFE_ACCESS_SIG1);
        ptr::write_volatile(REG_SAFE_ACCESS_SIG, SAFE_ACCESS_SIG2);

        riscv::asm::nop();
        riscv::asm::nop();
    }
    let ret = f();
    unsafe {
        ptr::write_volatile(REG_SAFE_ACCESS_SIG, 0);
        gintenr::write(gintenr::read() | (IRQ_STA & 0x08));
        IRQ_STA = 0;
        riscv::asm::nop();
        riscv::asm::nop();
    }
    ret
}

pub fn delay_us(t: u16) {
    let t = t as u32;
    let mut i = match sysctl::clocks().hclk.to_Hz() {
        60000000 => t * 15,
        80000000 => t * 20,
        48000000 => t * 12,
        32000000 => t * 8,
        24000000 => t * 6,
        16000000 => t * 4,
        8000000 => t * 2,
        4000000 => t,
        2000000 => t / 2,
        1000000 => t / 4,
        _ => t << 1, // default 2us
    };
    i = i / 8;
    unsafe {
        core::arch::asm!(
        "1:",
        "nop",
        "addi {0}, {0}, -1",
        "bne {0}, zero, 1b",
        inout(reg) i => _,
        options(nomem, nostack),
        );
    }
}

pub fn delay_ms(t: u16) {
    for _ in 0..t {
        delay_us(1000);
    }
}

#[derive(Debug, Clone, Default)]
pub struct Config {
    pub clock: sysctl::Config,
    /// All GPIO Input Pull Up, aka. HAL_SLEEP
    pub low_power: bool,
    /// Enable DCDC, aka. DCDC_ENABLE
    pub enable_dcdc: bool,
}

pub fn init(config: Config) -> Peripherals {
    let sys = unsafe { &*pac::SYS::PTR };
    if config.enable_dcdc {
        with_safe_access(|| {
            sys.aux_power_adj().modify(|_, w| w.dcdc_charge().set_bit());
            sys.power_plan().modify(|_, w| w.pwr_dcdc_pre().set_bit());
        });
        delay_us(10);
        with_safe_access(|| {
            sys.power_plan().modify(|_, w| w.pwr_dcdc_en().set_bit());
        });
    } else {
        with_safe_access(|| {
            sys.aux_power_adj().modify(|_, w| w.dcdc_charge().clear_bit());
            sys.power_plan()
                .modify(|_, w| w.pwr_dcdc_pre().clear_bit().pwr_dcdc_pre().clear_bit());
        });
    }

    config.clock.freeze();

    if config.low_power {
        unsafe {
            for rb in [&*pac::GPIOA::PTR, &*pac::GPIOB::PTR] {
                // in pu
                rb.pd_drv().write(|w| w.bits(0));
                rb.pu().write(|w| w.bits(0xffff));
                rb.dir().write(|w| w.bits(0));
            }
        }
    }

    Peripherals::take()
}

/// System reset
pub unsafe fn reset() -> ! {
    const KEY3: u16 = 0xBEEF;
    let pfic = unsafe { &*pac::PFIC::PTR };

    pfic.cfgr()
        .write(|w| unsafe { w.keycode().bits(KEY3).resetsys().set_bit() });

    loop {}
}

/// Software reset
pub unsafe fn soft_reset() -> ! {
    isp::flash_rom_reset();

    let rb = unsafe { &*pac::SYS::PTR };
    with_safe_access(|| {
        rb.rst_wdog_ctrl().modify(|_, w| w.software_reset().set_bit());
    });
    loop {}
}

pub static mut SERIAL: Option<uart::UartTx<peripherals::UART1>> = None;

#[macro_export]
macro_rules! println {
    ($($arg:tt)*) => {
        unsafe {
            use core::fmt::Write;
            use core::writeln;

            if let Some(uart) = $crate::SERIAL.as_mut() {
                writeln!(uart, $($arg)*).unwrap();
            }
        }
    }
}

pub unsafe fn set_default_serial(serial: uart::UartTx<'static, peripherals::UART1>) {
    SERIAL.replace(serial);
}

pub fn stack_free() -> usize {
    extern "C" {
        static mut _ebss: u32;
        static mut _stack_top: u32;
    }
    unsafe { &mut _stack_top as *mut u32 as usize - &mut _ebss as *mut u32 as usize }
}
