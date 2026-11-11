//! Writes to safe-access protected ("RWA") registers.
//!
//! Protected registers only accept a write within about 16 system clock cycles after
//! `0x57`, `0xA8` is written to `R8_SAFE_ACCESS_SIG` (CH583 datasheet, section 4.2).
//!
//! [`safe_modify`] and [`safe_write`] compute the new register value first, outside of
//! the window, using the usual PAC closures. Only the final store happens inside the
//! window, in a fixed `asm!` sequence. This makes the timing independent of the
//! optimization level, so it also works in debug builds.

use ch58x::{Readable, Reg, RegisterSpec, Resettable, Writable, R, W};

const REG_SAFE_ACCESS_SIG: *mut u8 = 0x4000_1040 as *mut u8;

/// Register widths that can be stored inside a safe-access window.
pub trait SafeAccessWord: Copy {
    /// Unlock, store `value` at `addr`, and lock again.
    ///
    /// # Safety
    /// `addr` must be a valid, protected peripheral register of this width.
    unsafe fn safe_store(addr: *mut Self, value: Self);
}

macro_rules! impl_safe_access_word {
    ($t:ty, $store:literal) => {
        impl SafeAccessWord for $t {
            #[inline(always)]
            unsafe fn safe_store(addr: *mut Self, value: Self) {
                // Same sequence as `sys_safe_access_enable/disable` in the WCH SDK, with
                // the store in between and nothing else.
                core::arch::asm!(
                    // disable interrupts (GINTENR), remember whether they were enabled
                    "csrrci {irq}, 0x800, 0x8",
                    "fence.i",
                    "sb {sig1}, 0({sig})",
                    "sb {sig2}, 0({sig})",
                    "fence.i",
                    concat!($store, " {value}, 0({addr})"),
                    // the SDK keeps a few cycles in the window after the store
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    // leave safe access mode, restore interrupts
                    "sb zero, 0({sig})",
                    "andi {irq}, {irq}, 0x8",
                    "csrs 0x800, {irq}",
                    "fence.i",
                    irq = out(reg) _,
                    sig = in(reg) REG_SAFE_ACCESS_SIG,
                    sig1 = in(reg) 0x57u32,
                    sig2 = in(reg) 0xA8u32,
                    // sb/sh store only the low 8/16 bits
                    value = in(reg) value as u32,
                    addr = in(reg) addr,
                    options(nostack),
                );
            }
        }
    };
}

impl_safe_access_word!(u8, "sb");
impl_safe_access_word!(u16, "sh");
impl_safe_access_word!(u32, "sw");

/// Like `reg.modify(f)`, for a safe-access protected register.
///
/// ```ignore
/// safe_modify(sys.power_plan(), |_, w| w.pwr_plan_en().set_bit());
/// ```
#[inline]
pub fn safe_modify<REG, F>(reg: &Reg<REG>, f: F) -> REG::Ux
where
    REG: Readable + Writable,
    REG::Ux: SafeAccessWord,
    for<'w> F: FnOnce(&R<REG>, &'w mut W<REG>) -> &'w mut W<REG>,
{
    // Keep read-modify-write atomic: an interrupt between the read and the store must not
    // change the register, or its change would be overwritten with the stale value.
    critical_section::with(|_| {
        let value = with_shadow(reg.read().bits(), |shadow| shadow.modify(f));
        unsafe { REG::Ux::safe_store(reg.as_ptr(), value) };
        value
    })
}

/// Like `reg.write(f)`, for a safe-access protected register.
///
/// ```ignore
/// safe_write(sys.flash_cfg(), |w| unsafe { w.bits(0x52) });
/// ```
#[inline]
pub fn safe_write<REG, F>(reg: &Reg<REG>, f: F) -> REG::Ux
where
    REG: Resettable + Writable,
    REG::Ux: SafeAccessWord,
    F: FnOnce(&mut W<REG>) -> &mut W<REG>,
{
    // `write` starts from the reset value, the initial shadow content is irrelevant
    let value = with_shadow(REG::RESET_VALUE, |shadow| shadow.write(f));
    unsafe { REG::Ux::safe_store(reg.as_ptr(), value) };
    value
}

/// Runs `f` on a copy of a register in RAM, initialized with `bits`, and returns its result.
///
/// The PAC closures can then be evaluated without touching the hardware register.
#[inline(always)]
fn with_shadow<REG, T>(bits: REG::Ux, f: impl FnOnce(&Reg<REG>) -> T) -> T
where
    REG: RegisterSpec,
{
    let shadow = core::cell::UnsafeCell::new(bits);
    // SAFETY: `Reg<REG>` is `#[repr(transparent)]` over `VolatileCell<REG::Ux>`, which is
    // `#[repr(transparent)]` over `UnsafeCell<REG::Ux>`, so this has the same layout.
    let shadow_reg = unsafe { &*(shadow.get() as *const Reg<REG>) };
    f(shadow_reg)
}
