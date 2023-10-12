//! Segment LCD controller

use crate::pac;
use pac::LCD;

pub struct Config {
    pub voltage: bool,
    pub scan_clk: u8,
    pub duty: u8,
    pub bias: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            voltage: false,
            scan_clk: 0b11,
            duty: 0b10,
            bias: true,
        }
    }
}

impl Config {
    pub fn voltage3v3(mut self) -> Self {
        self.voltage = false;
        self
    }
    pub fn voltage2v7(mut self) -> Self {
        self.voltage = true;
        self
    }
    pub fn duty1_4(mut self) -> Self {
        self.duty = 0b10;
        self
    }
}

pub struct Lcd {
    //lcd: LCD,
}

impl Lcd {
    pub fn new(_lcd: LCD, config: Config) -> Self {
        let lcd = unsafe { &*pac::LCD::PTR };
        let sys = unsafe { &*pac::SYS::PTR };

        // disable digital input
        unsafe {
            sys.pin_config2.modify(|_, w| w.bits(0xfffeff3f));
        }

        // default on
        sys.slp_clk_off1.modify(|_, w| w.slp_clk_lcd().clear_bit());

        // lcd config
        lcd.cmd.write(|w| unsafe {
            w.seg_en()
                .bits(0x1ffff)
                .vlcd_sel()
                .bit(config.voltage) // 2.5V
                .scan_clk_sel()
                .bits(config.scan_clk)
                .duty()
                .bits(config.duty) // 1/4
                .bias()
                .bit(config.bias)
                .on()
                .set_bit()
                .sys_en()
                .set_bit()
        });
        lcd.ram0.reset();
        lcd.ram1.reset();
        lcd.ram2.reset();

        Self {}
    }

    // n: 0 to 10
    pub fn write_data(&self, n: u8, data: u8) {
        if n < 4 {
            let lcd = unsafe { &*pac::LCD::PTR };
            let data = (data as u32) << (n * 8);
            lcd.ram0
                .modify(|r, w| unsafe { w.bits((r.bits() & !(0xff << (n * 8))) | data) });
        } else if n < 8 {
            let lcd = unsafe { &*pac::LCD::PTR };
            let data = (data as u32) << ((n - 4) * 8);
            lcd.ram1
                .modify(|r, w| unsafe { w.bits((r.bits() & !(0xff << ((n - 4) * 8))) | data) });
            return;
        } else if n < 12 {
            let lcd = unsafe { &*pac::LCD::PTR };
            let data = (data as u32) << ((n - 8) * 8);
            lcd.ram2
                .modify(|r, w| unsafe { w.bits((r.bits() & !(0xff << ((n - 8) * 8))) | data) });
            return;
        }
    }

    pub fn write_ram0(&self, data: u32) {
        let lcd = unsafe { &*pac::LCD::PTR };
        lcd.ram0.write(|w| unsafe { w.bits(data) });
    }

    pub fn write_ram1(&self, data: u32) {
        let lcd = unsafe { &*pac::LCD::PTR };
        lcd.ram1.write(|w| unsafe { w.bits(data) });
    }

    pub fn write_ram2(&self, data: u32) {
        let lcd = unsafe { &*pac::LCD::PTR };
        lcd.ram2.write(|w| unsafe { w.bits(data & 0xffff) });
    }
}
