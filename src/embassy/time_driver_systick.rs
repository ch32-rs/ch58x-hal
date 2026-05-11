//! SysTick-based time driver.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::Waker;

use critical_section::{CriticalSection, Mutex};
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;

// use super::AlarmState;
use crate::pac;

pub struct SystickDriver {
    period: AtomicU32,
    queue: Mutex<RefCell<Queue>>,
}

embassy_time_driver::time_driver_impl!(static DRIVER: SystickDriver = SystickDriver {
    period: AtomicU32::new(1), // avoid div by zero
    queue: Mutex::new(RefCell::new(Queue::new())),
});

impl SystickDriver {
    fn init(&'static self) {
        let rb = unsafe { &*pac::SYSTICK::PTR };
        let hclk = crate::sysctl::clocks().hclk.to_Hz() as u64;

        let cnt_per_second = hclk / 8;
        let cnt_per_tick = cnt_per_second / embassy_time_driver::TICK_HZ;

        self.period.store(cnt_per_tick as u32, Ordering::Relaxed);

        // UNDOCUMENTED:  Avoid initial interrupt
        rb.cmp().write(|w| unsafe { w.bits(u64::MAX - 1) });
        critical_section::with(|_| {
            rb.sr().write(|w| w.cntif().bit(false)); // clear
                                                     // Configration: Upcount, No reload, HCLK/8 as clock source
            rb.ctlr().modify(|_, w| {
                w.init()
                    .set_bit()
                    .mode()
                    .upcount()
                    .stre()
                    .clear_bit()
                    .stclk()
                    .hclk_div8()
                    .ste()
                    .set_bit()
            });
        })
    }

    fn on_interrupt(&self) {
        critical_section::with(|cs| {
            let systick = unsafe { &*pac::SYSTICK::PTR };
            systick.sr().write(|w| w.cntif().clear_bit()); // clear interrupt flag

            let mut queue = self.queue.borrow_ref_mut(cs);
            self.update_alarms(cs, &mut queue);
        });
    }

    fn update_alarms(&self, _: CriticalSection, queue: &mut Queue) -> bool {
        let systick = unsafe { &*pac::SYSTICK::PTR };

        loop {
            // Wakes all tasks scheduled now or before
            let next_alarm = queue.next_expiration(self.now());

            // No more scheduled tasks
            if next_alarm == u64::MAX {
                // Disabling the interrupt is unnecesary but may (untested) save power.
                systick.ctlr().modify(|_, w| w.stie().clear_bit());
                return false;
            }

            // TODO: why did the old hal use an atomic
            let cmp_timestamp = next_alarm * self.period.load(Ordering::Relaxed) as u64; // fails
            systick.cmp().write(|w| unsafe { w.bits(cmp_timestamp) });
            systick.ctlr().modify(|_, w| w.stie().set_bit()); // set interrupt

            // UNDOCUMENTED: The timer will only interrupt when cnt and cmp match exactly.
            // If cmp is now less than cnt, there is a good chance the flag was not
            // and will never be triggered.
            if systick.cmp().read().bits() > systick.cnt().read().bits() {
                return true;
            }

            // If cmp < count, loop and wake the task now. Ensure the flag is unset
            // to avoid a potential extra interrupt.
            systick.sr().write(|w| w.cntif().clear_bit());
        }
    }
}

impl Driver for SystickDriver {
    fn now(&self) -> u64 {
        let rb = unsafe { &*pac::SYSTICK::PTR };
        rb.cnt().read().bits() / (self.period.load(Ordering::Relaxed) as u64)
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow_ref_mut(cs);
            if queue.schedule_wake(at, waker) {
                self.update_alarms(cs, &mut queue);
            }
        });
    }
}

#[qingke_rt::interrupt]
fn SysTick() {
    DRIVER.on_interrupt();
}

pub(crate) fn init() {
    use qingke::interrupt::Priority;
    use qingke_rt::CoreInterrupt;

    DRIVER.init();

    unsafe {
        qingke::pfic::set_priority(CoreInterrupt::SysTick as u8, Priority::P15 as _);
        qingke::pfic::enable_interrupt(CoreInterrupt::SysTick as u8);
    }
}
