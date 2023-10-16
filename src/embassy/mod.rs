use core::cell::Cell;

use self::time_driver::EmbassyTimer;

mod time_driver;

pub struct AlarmState {
    pub timestamp: Cell<u64>,
    pub callback: Cell<Option<(fn(*mut ()), *mut ())>>,
    pub allocated: Cell<bool>,
}

unsafe impl Send for AlarmState {}

impl AlarmState {
    pub const fn new() -> Self {
        Self {
            timestamp: Cell::new(0),
            callback: Cell::new(None),
            allocated: Cell::new(false),
        }
    }
}
