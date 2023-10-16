use critical_section::{CriticalSection, Mutex};

use super::AlarmState;

pub const ALARM_COUNT: usize = 1;

pub struct EmbassyTimer {
    pub(crate) alarms: Mutex<[AlarmState; ALARM_COUNT]>,
}
