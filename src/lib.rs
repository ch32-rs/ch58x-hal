#![no_std]
#![recursion_limit = "1024"]
use core::ptr;

pub use ch59x::ch59x as pac;

pub use self::peripheral::{Peripheral, PeripheralRef};
pub use self::peripherals::Peripherals;

pub mod adc;
pub mod dma;
pub mod gpio;
pub mod i2c;
pub mod lcd;
pub mod rtc;
pub mod signature;
pub mod spi;
pub mod sysctl;
pub mod systick;
pub mod timer;
pub mod uart;

// #[cfg(feature = "isp")]
pub mod interrupt;
pub mod isp;
pub mod rt;
pub(crate) mod traits;

pub mod peripherals;

pub mod peripheral;
pub mod prelude;

mod critical_section;

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

pub struct Config {
    pub clock: sysctl::Config,
    /// All GPIO Input Pull Up
    pub low_power: bool,
    /// Enable DCDC
    pub enable_dcdc: bool,
}

pub fn init(_config: Config) -> Peripherals {
    todo!()
}

// pin trait

macro_rules! pin_trait_impl {
    (crate::$mod:ident::$trait:ident, $instance:ident, $pin:ident, $remap:expr) => {
        impl crate::$mod::$trait<crate::peripherals::$instance> for crate::peripherals::$pin {
            fn is_remap(&self) -> bool {
                $remap
            }
        }
    };
}

pin_trait_impl!(crate::uart::TxPin, UART0, PB7, false);
pin_trait_impl!(crate::uart::TxPin, UART0, PA14, true);

pin_trait_impl!(crate::uart::TxPin, UART1, PA9, false);
pin_trait_impl!(crate::uart::TxPin, UART1, PB13, true);

pin_trait_impl!(crate::uart::TxPin, UART2, PA7, false);
pin_trait_impl!(crate::uart::TxPin, UART2, PB23, true);

pin_trait_impl!(crate::uart::TxPin, UART3, PA5, false);
pin_trait_impl!(crate::uart::TxPin, UART3, PB21, true);
