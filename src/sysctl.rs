use fugit::HertzU32 as Hertz;

use crate::pac::{SYS, SYSCTL};
use crate::with_safe_access;

// No HSI
const HSE_FREQUENCY: Hertz = Hertz::from_raw(32_000_000);
const PLL_FREQUENCY: Hertz = Hertz::from_raw(480_000_000);

static mut CLOCK: Clocks = Clocks {
    // Power on default
    hclk: Hertz::from_raw(6_400_000),
};

/// 32K clock source
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum Clock32KSrc {
    #[default]
    LSI,
    LSE,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(u8)]
pub enum ClockSrc {
    // CK32K
    Clock32K,
    // CK32M from HSE, then div, 2 <= div <= 32
    HSE(u8),
    // CK32M from PLL, then div, 2 <= div <= 32
    PLL(u8),
}

impl Default for ClockSrc {
    fn default() -> Self {
        Self::PLL(8)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Config {
    pub clock32ksrc: Clock32KSrc,
    pub mux: ClockSrc,
}

impl Config {
    pub fn use_lsi_32k() -> Self {
        Self {
            mux: ClockSrc::Clock32K,
            clock32ksrc: Clock32KSrc::LSI,
        }
    }

    pub fn use_lse_32k() -> Self {
        Self {
            mux: ClockSrc::Clock32K,
            clock32ksrc: Clock32KSrc::LSE,
        }
    }

    pub fn pll_60mhz() -> Self {
        Self {
            mux: ClockSrc::PLL(8),
            ..Default::default()
        }
    }

    pub fn pll_80mhz() -> Self {
        Self {
            mux: ClockSrc::PLL(6),
            ..Default::default()
        }
    }

    pub fn pll_48mhz() -> Self {
        Self {
            mux: ClockSrc::PLL(10),
            ..Default::default()
        }
    }

    pub fn pll_32mhz() -> Self {
        Self {
            mux: ClockSrc::PLL(15),
            ..Default::default()
        }
    }

    pub fn pll_24mhz() -> Self {
        Self {
            mux: ClockSrc::PLL(20),
            ..Default::default()
        }
    }

    pub fn enable_lse(mut self) -> Self {
        self.clock32ksrc = Clock32KSrc::LSE;
        self
    }

    pub fn freeze(self) {
        let sysctl = unsafe { &*SYSCTL::PTR };
        let sys = unsafe { &*SYS::PTR };

        match self.clock32ksrc {
            Clock32KSrc::LSE => {
                with_safe_access(|| {
                    sys.ck32k_config.modify(|_, w| w.clk_xt32k_pon().set_bit());
                });
                unsafe {
                    riscv::asm::delay(clocks().hclk.to_Hz() / 10 / 4);
                }
                with_safe_access(|| unsafe {
                    sys.xt32k_tune.modify(|_, w| w.xt32k_i_tune().bits(0b01));
                });
                with_safe_access(|| {
                    sys.ck32k_config.modify(|_, w| w.clk_osc32k_xt().set_bit());
                });
                unsafe {
                    riscv::asm::delay(clocks().hclk.to_Hz() / 1000);
                }
            }
            _ => (),
        }

        with_safe_access(|| unsafe {
            sysctl.pll_config.modify(|r, w| w.bits(r.bits() & !(1 << 5)));
        });
        let hclk = match self.mux {
            ClockSrc::HSE(div) => {
                assert!(div != 1, "1 means close HCLK");
                with_safe_access(|| unsafe {
                    sys.clk_sys_cfg.write(|w| {
                        w.pll_pwr_en()
                            .set_bit()
                            .xt_32m_pwr_en()
                            .set_bit()
                            .clk_sys_mod()
                            .bits(0b10)
                            .clk_pll_div()
                            .bits(div & 0x1f)
                    });
                    riscv::asm::nop();
                    riscv::asm::nop();
                    riscv::asm::nop();
                    riscv::asm::nop();
                });
                with_safe_access(|| unsafe {
                    sysctl.flash_cfg.modify(|_, w| w.bits(0x51));
                });
                Hertz::from_raw(HSE_FREQUENCY.to_Hz() / (div as u32))
            }
            ClockSrc::PLL(div) => {
                assert!(div != 1, "1 means close HCLK");
                with_safe_access(|| unsafe {
                    sys.clk_sys_cfg.write(|w| {
                        w.pll_pwr_en()
                            .set_bit()
                            .xt_32m_pwr_en()
                            .set_bit()
                            .clk_sys_mod()
                            .bits(0b01)
                            .clk_pll_div()
                            .bits(div & 0x1f)
                    });
                    riscv::asm::nop();
                    riscv::asm::nop();
                    riscv::asm::nop();
                    riscv::asm::nop();
                });
                with_safe_access(|| unsafe {
                    sysctl.flash_cfg.modify(|_, w| w.bits(0x52));
                });
                Hertz::from_raw(PLL_FREQUENCY.to_Hz() / (div as u32))
            }
            _ => {
                sys.clk_sys_cfg.modify(|r, w| unsafe { w.bits(r.bits() | 0xC0) });
                Hertz::from_raw(32_768)
            }
        };
        with_safe_access(|| unsafe {
            sysctl.pll_config.modify(|r, w| w.bits(r.bits() | (1 << 7)));
        });

        unsafe {
            CLOCK = Clocks { hclk };
        }
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct Clocks {
    pub hclk: Hertz,
}

pub fn clocks() -> &'static Clocks {
    unsafe { &CLOCK }
}
