use core::{
    mem, ptr,
    sync::atomic::{compiler_fence, Ordering},
};

use crate::pac;
use crate::rt::Interrupt as InterruptEnum;

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
    /// Return the interrupt number associated with this variant.
    ///
    /// See trait documentation for safety requirements.
    fn number(self) -> u16;
}

unsafe impl InterruptNumber for InterruptEnum {
    #[inline]
    fn number(self) -> u16 {
        self as u16
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
    SysTick, Software, TMR0, GPIOA, GPIOB, SPI0, BLEL, BLEB, USB, // USB2,
    TMR1, TMR2, UART0, UART1, RTC,
);

/// Represents an interrupt type that can be configured by embassy to handle
/// interrupts.
pub unsafe trait InterruptExt: InterruptNumber + Copy {
    /// Enable the interrupt.
    #[inline]
    unsafe fn enable(self) {
        compiler_fence(Ordering::SeqCst);
        let pfic = &*pac::PFIC::PTR;
        if self.number() < 32 {
            // Write 1 to enable, 0 unchanged
            pfic.ienr1.write(|w| w.bits(1 << self.number()));
        } else {
            pfic.ienr2.write(|w| w.bits(1 << (self.number() % 32)));
        }
    }

    /// Disable the interrupt.
    #[inline]
    fn disable(self) {
        unsafe {
            let pfic = &*pac::PFIC::PTR;
            if self.number() < 32 {
                // Write 1 to close interrupt, 0 unchanged
                pfic.irer1.write(|w| w.bits(1 << self.number()));
            } else {
                pfic.irer2.write(|w| w.bits(1 << (self.number() % 32)));
            }
        }
    }

    // Check if interrupt is being handled.
    #[inline]
    #[cfg(not(armv6m))]
    fn is_active(self) -> bool {
        let pfic = unsafe { &*pac::PFIC::PTR };
        if self.number() < 32 {
            pfic.iactr1.read().bits() & (1 << self.number()) != 0
        } else {
            pfic.iactr2.read().bits() & (1 << (self.number() % 32)) != 0
        }
    }

    // NOTE: no check support
    //#[inline]
    //fn is_enabled(self) -> bool {
    //    NVIC::is_enabled(self)
    //}
    // Check if interrupt is pending.
    //#[inline]
    //fn is_pending(self) -> bool {
    //    NVIC::is_pending(self)
    //}

    /// Set interrupt pending.
    #[inline]
    fn pend(self) {
        unsafe {
            let pfic = &*pac::PFIC::PTR;
            if self.number() < 32 {
                // Write 1 to close interrupt, 0 unchanged
                pfic.ipsr1.write(|w| w.bits(1 << self.number()));
            } else {
                pfic.ipsr2.write(|w| w.bits(1 << (self.number() % 32)));
            }
        }
    }

    /// Unset interrupt pending.
    #[inline]
    fn unpend(self) {
        unsafe {
            let pfic = &*pac::PFIC::PTR;
            if self.number() < 32 {
                // Write 1 to close interrupt, 0 unchanged
                pfic.iprr1.write(|w| w.bits(1 << self.number()));
            } else {
                pfic.iprr2.write(|w| w.bits(1 << (self.number() % 32)));
            }
        }
    }

    /// Get the priority of the interrupt.
    #[inline]
    fn get_priority(self) -> Priority {
        const PFIC_IACTR: *mut u8 = 0xE000E400 as *mut u8;
        let raw = unsafe { ptr::read_volatile(PFIC_IACTR.offset(self.number() as isize)) };
        Priority::from(raw)
    }

    /// Set the interrupt priority.
    #[inline]
    fn set_priority(self, prio: Priority) {
        const PFIC_IACTR: *mut u8 = 0xE000E400 as *mut u8;
        let raw_ptr = unsafe { PFIC_IACTR.offset(self.number() as isize) };
        critical_section::with(|_| unsafe {
            ptr::write_volatile(raw_ptr, prio as u8);
        })
    }
}

const PRIO_MASK: u8 = 0xf0;

unsafe impl<T: InterruptNumber + Copy> InterruptExt for T {}

impl From<u8> for Priority {
    fn from(priority: u8) -> Self {
        unsafe { mem::transmute(priority & PRIO_MASK) }
    }
}

impl From<Priority> for u8 {
    fn from(p: Priority) -> Self {
        p as u8
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
#[allow(missing_docs)]
pub enum Priority {
    P0 = 0x0,
    P1 = 0x10,
    P2 = 0x20,
    P3 = 0x30,
    P4 = 0x40,
    P5 = 0x50,
    P6 = 0x60,
    P7 = 0x70,
    P8 = 0x80,
    P9 = 0x90,
    P10 = 0xa0,
    P11 = 0xb0,
    P12 = 0xc0,
    P13 = 0xd0,
    P14 = 0xe0,
    P15 = 0xf0,
}
