//! Checks that writes to safe-access protected registers take effect.
//!
//! Protected registers (clock, power, RTC mode, ...) only accept a write within a short window
//! after unlocking `R8_SAFE_ACCESS_SIG`. If the window is missed, the write is silently dropped.
//!
//! This example does not use UART on purpose: if the clock setup fails, the baud rate is wrong
//! too. Instead, the results are stored in `SAFE_ACCESS_RESULT` and read with a debugger:
//!
//! ```text
//! openocd -f wch-riscv.cfg -c init -c halt -c "mdw <addr of SAFE_ACCESS_RESULT> 12" -c resume -c shutdown
//! ```
//!
//! Layout (u32 words):
//! - `[0]` magic `0x5AFEACCE` once all checks ran
//! - `[1]` progress: last step reached (see `Step`), useful if something hangs
//! - `[2]` pass mask, `[3]` expected mask (all checks passed if equal)
//! - `[4..]` raw register values, see below
//!
//! Build it in both debug and release mode. Set `USE_LSE` on boards with a 32.768 kHz crystal.
//!
//! Note: the `pll_config` check, and the `ck32k_config` check with the internal 32 kHz oscillator,
//! expect values that equal the reset state, so they also pass if the writes are dropped.
//! With `USE_LSE`, the `ck32k_config` check is meaningful.

#![no_std]
#![no_main]

use core::ptr::{addr_of_mut, write_volatile};

use hal::rtc::{DateTime, Rtc, TimingMode};
use {ch58x_hal as hal, panic_halt as _};

/// Use the external 32.768 kHz crystal instead of the internal RC oscillator.
const USE_LSE: bool = false;

#[no_mangle]
static mut SAFE_ACCESS_RESULT: [u32; 12] = [0; 12];

#[repr(u32)]
enum Step {
    Init = 1,
    RtcSet = 2,
    RtcWait = 3,
    RtcTiming = 4,
    Done = 5,
}

const CHECK_CLK_SYS_CFG: u32 = 1 << 0;
const CHECK_FLASH_CFG: u32 = 1 << 1;
const CHECK_PLL_CONFIG: u32 = 1 << 2;
const CHECK_CK32K_CONFIG: u32 = 1 << 3;
const CHECK_RTC_SET: u32 = 1 << 4;
const CHECK_RTC_TIMING_ON: u32 = 1 << 5;
const CHECK_RTC_TIMING_OFF: u32 = 1 << 6;
const ALL_CHECKS: u32 = (1 << 7) - 1;

fn store(idx: usize, value: u32) {
    unsafe { write_volatile(&mut (*addr_of_mut!(SAFE_ACCESS_RESULT))[idx], value) };
}

#[qingke_rt::entry]
fn main() -> ! {
    store(1, Step::Init as u32);

    // PLL / 8 = 60 MHz
    let mut config = hal::Config::default();
    config.clock.use_pll_60mhz();
    if USE_LSE {
        config.clock.enable_lse();
    }
    let p = hal::init(config);

    let sys = unsafe { &*hal::pac::SYS::PTR };
    let rtc_regs = unsafe { &*hal::pac::RTC::PTR };
    let mut pass = 0;

    // [4] clk_sys_cfg: clk_sys_mod = 0b01 (PLL), clk_pll_div = 8
    let clk_sys_cfg = sys.clk_sys_cfg().read().bits() as u32;
    store(4, clk_sys_cfg);
    if clk_sys_cfg == (0b01 << 6) | 8 {
        pass |= CHECK_CLK_SYS_CFG;
    }

    // [5] flash_cfg: 0x52 for every PLL frequency except 80 MHz
    let flash_cfg = sys.flash_cfg().read().bits() as u32;
    store(5, flash_cfg);
    if flash_cfg == 0x52 {
        pass |= CHECK_FLASH_CFG;
    }

    // [6] pll_config: freeze() clears bit 5 first and sets bit 7 at the end
    let pll_config = sys.pll_config().read().bits() as u32;
    store(6, pll_config);
    if pll_config & (1 << 7) != 0 && pll_config & (1 << 5) == 0 {
        pass |= CHECK_PLL_CONFIG;
    }

    // [7] ck32k_config: 32 kHz source selection
    let ck32k = sys.ck32k_config().read();
    store(7, ck32k.bits() as u32);
    let ck32k_ok = if USE_LSE {
        ck32k.clk_xt32k_pon().bit_is_set() && ck32k.clk_osc32k_xt().bit_is_set()
    } else {
        ck32k.clk_int32k_pon().bit_is_set() && ck32k.clk_osc32k_xt().bit_is_clear()
    };
    if ck32k_ok {
        pass |= CHECK_CK32K_CONFIG;
    }

    // RTC: set the time (trig + load_hi/load_lo), then read it back
    let mut rtc = Rtc::new(p.RTC);
    store(1, Step::RtcSet as u32);
    rtc.set_datatime(DateTime {
        year: 2026,
        month: 11,
        day: 11,
        hour: 12,
        minute: 34,
        second: 50,
        millisecond: 0,
    });
    store(1, Step::RtcWait as u32);
    // ~0.5 s, so the RTC visibly counts
    hal::delay_ms(500);
    let now = rtc.now();
    // [8] y/m/d packed, [9] h/m/s packed
    store(8, (now.year as u32) << 16 | (now.month as u32) << 8 | now.day as u32);
    store(
        9,
        (now.hour as u32) << 16 | (now.minute as u32) << 8 | now.second as u32,
    );
    if (now.year, now.month, now.day, now.hour, now.minute) == (2026, 11, 11, 12, 34) && (50..=52).contains(&now.second)
    {
        pass |= CHECK_RTC_SET;
    }

    // RTC timing mode (mode_ctrl: tmr_mode + tmr_en)
    store(1, Step::RtcTiming as u32);
    rtc.enable_timing(TimingMode::_1S);
    let mode_on = rtc_regs.mode_ctrl().read();
    store(10, mode_on.bits() as u32);
    if mode_on.tmr_en().bit_is_set() && mode_on.tmr_mode().bits() == TimingMode::_1S as u8 {
        pass |= CHECK_RTC_TIMING_ON;
    }
    rtc.disable_timing();
    let mode_off = rtc_regs.mode_ctrl().read();
    store(11, mode_off.bits() as u32);
    // only meaningful if enabling worked, otherwise tmr_en was never set
    if pass & CHECK_RTC_TIMING_ON != 0 && mode_off.tmr_en().bit_is_clear() {
        pass |= CHECK_RTC_TIMING_OFF;
    }

    store(2, pass);
    store(3, ALL_CHECKS);
    store(1, Step::Done as u32);
    store(0, 0x5AFE_ACCE);

    loop {
        core::hint::spin_loop();
    }
}
