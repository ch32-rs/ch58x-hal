use critical_section::{set_impl, Impl, RawRestoreState};
use qingke::register::gintenr;

struct SingleHartCriticalSection;
set_impl!(SingleHartCriticalSection);

unsafe impl Impl for SingleHartCriticalSection {
    unsafe fn acquire() -> RawRestoreState {
        let irq_state = gintenr::read();
        if irq_state & 0x08 != 0 {
            gintenr::write(irq_state & (!0x08));
        }
        irq_state as u8 // as the high bits are reserved 0x0.
    }

    unsafe fn release(irq_state: RawRestoreState) {
        // Only re-enable interrupts if they were enabled before the critical section.
        let irq_state = irq_state as usize;
        gintenr::write(gintenr::read() | (irq_state & 0x08));
    }
}
