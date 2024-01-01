use core::sync::atomic::{compiler_fence, Ordering};
use core::{mem, ptr};

pub use qingke::interrupt::Priority;
use qingke_rt::CoreInterrupt as CoreInterruptEnum;

use crate::pac;
use crate::pac::interrupt::Interrupt as InterruptEnum;
use crate::pac::__EXTERNAL_INTERRUPTS as _;

/// Trait for enums of external interrupt numbers.
///
/// This trait should be implemented by a peripheral access crate (PAC)
/// on its enum of available external interrupts for a specific device.
/// Each variant must convert to a u16 of its interrupt number,
/// which is its exception number - 16.
///
/// # Safety
///
/// This trait must only be implemented on enums of device interrupts. Each
/// enum variant must represent a distinct value (no duplicates are permitted),
/// and must always return the same value (do not change at runtime).
///
/// These requirements ensure safe nesting of critical sections.
pub unsafe trait InterruptNumber: Copy {
    /// Highest number assigned to an interrupt source.
    const MAX_INTERRUPT_NUMBER: u8;

    /// Converts an interrupt source to its corresponding number.
    fn number(self) -> u8;

    /// Tries to convert a number to a valid interrupt source.
    /// If the conversion fails, it returns an error with the number back.
    fn from_number(value: u8) -> Result<Self, u8>;
}

unsafe impl InterruptNumber for InterruptEnum {
    const MAX_INTERRUPT_NUMBER: u8 = 35;

    fn number(self) -> u8 {
        self as u8
    }

    fn from_number(value: u8) -> Result<Self, u8> {
        InterruptEnum::try_from(value).map_err(|_| value)
    }
}

mod sealed {
    pub trait Interrupt {}
}

/// Type-level interrupt.
///
/// This trait is implemented for all typelevel interrupt types in this module.
pub trait Interrupt: sealed::Interrupt {
    /// Interrupt enum variant.
    ///
    /// This allows going from typelevel interrupts (one type per interrupt) to
    /// non-typelevel interrupts (a single `Interrupt` enum type, with one variant per interrupt).
    const IRQ: InterruptEnum;

    /// Enable the interrupt.
    #[inline]
    unsafe fn enable() {
        Self::IRQ.enable()
    }

    /// Disable the interrupt.
    #[inline]
    fn disable() {
        Self::IRQ.disable()
    }

    /*
    /// Check if interrupt is enabled.
    #[inline]
    fn is_enabled() -> bool {
        Self::IRQ.is_enabled()
    }

    /// Check if interrupt is pending.
    #[inline]
    fn is_pending() -> bool {
        Self::IRQ.is_pending()
    }
    */

    /// Set interrupt pending.
    #[inline]
    fn pend() {
        Self::IRQ.pend()
    }

    /// Unset interrupt pending.
    #[inline]
    fn unpend() {
        Self::IRQ.unpend()
    }

    /// Get the priority of the interrupt.
    #[inline]
    fn get_priority() -> crate::interrupt::Priority {
        Self::IRQ.get_priority()
    }

    /// Set the interrupt priority.
    #[inline]
    fn set_priority(prio: crate::interrupt::Priority) {
        Self::IRQ.set_priority(prio)
    }
}

macro_rules! impl_irqs {
    ($($irqs:ident),* $(,)?) => {
        $(
            #[allow(non_camel_case_types)]
            #[doc=stringify!($irqs)]
            #[doc=" typelevel interrupt."]
            pub enum $irqs {}
            impl sealed::Interrupt for $irqs{}
            impl Interrupt for $irqs {
                const IRQ: InterruptEnum = InterruptEnum::$irqs;
            }
        )*
    }
}

impl_irqs!(
    TMR0, GPIOA, GPIOB, SPI0, BLEL, BLEB, USB, // USB2,
    TMR1, TMR2, UART0, UART1, UART2, UART3, RTC, I2C, ADC,
);

/// Represents an interrupt type that can be configured by embassy to handle
/// interrupts.
pub unsafe trait InterruptExt: InterruptNumber + Copy {
    /// Enable the interrupt.
    #[inline]
    unsafe fn enable(self) {
        qingke::pfic::enable_interrupt(self.number())
    }

    /// Disable the interrupt.
    #[inline]
    fn disable(self) {
        unsafe { qingke::pfic::disable_interrupt(self.number()) }
    }

    // Check if interrupt is being handled.
    #[inline]
    fn is_active(self) -> bool {
        qingke::pfic::is_active(self.number())
    }

    /// Set interrupt pending.
    #[inline]
    fn pend(self) {
        unsafe { qingke::pfic::pend_interrupt(self.number()) }
    }

    /// Unset interrupt pending.
    #[inline]
    fn unpend(self) {
        unsafe { qingke::pfic::unpend_interrupt(self.number()) }
    }

    /// Get the priority of the interrupt.
    #[inline]
    fn get_priority(self) -> Priority {
        qingke::pfic::get_priority(self.number()).into()
    }

    /// Set the interrupt priority.
    #[inline]
    fn set_priority(self, prio: Priority) {
        unsafe { qingke::pfic::set_priority(self.number(), prio.into()) }
    }
}

const PRIO_MASK: u8 = 0xf0;

unsafe impl<T: InterruptNumber + Copy> InterruptExt for T {}
