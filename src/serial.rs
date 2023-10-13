//! UART: Uni

use crate::pac;

// Default UART is UART1(PA8/PA9)

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Parity {
    ParityNone = 0xff,
    ParityEven = 0b01,
    ParityOdd = 0b00,
    ParityMark = 0b10,  // 1
    ParitySpace = 0b11, // 0
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum StopBits {
    #[doc = "1 stop bit"]
    STOP1,
    #[doc = "2 stop bits"]
    STOP2,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DataBits {
    DataBits5 = 0b00,
    DataBits6 = 0b01,
    DataBits7 = 0b10,
    DataBits8 = 0b11,
}

pub struct Config {
    pub baudrate: u32,
    pub data_bits: DataBits,
    pub stop_bits: StopBits,
    pub parity: Parity,
}
impl Default for Config {
    fn default() -> Self {
        Self {
            baudrate: 115200,
            data_bits: DataBits::DataBits8,
            stop_bits: StopBits::STOP1,
            parity: Parity::ParityNone,
        }
    }
}

pub struct Uart {}

impl Uart {
    pub fn new(config: Config) -> Self {
        let uart1 = unsafe { &*pac::UART1::PTR };
        let _sys = unsafe { &*pac::SYS::PTR };

        // default on
        // sys.slp_clk_off0
        //    .modify(|_, w| w.slp_clk_uart1().clear_bit());

        uart1.fcr.write(|w| unsafe {
            w.rx_fifo_clr()
                .set_bit()
                .rx_fifo_clr()
                .set_bit()
                .fifo_en() // enable 8 byte FIFO
                .set_bit()
                .fifo_trig()
                .bits(2) // 4 bytes
        });
        uart1.lcr.write(|w| unsafe { w.word_sz().bits(config.data_bits as u8) }); // 8 bits
        match config.stop_bits {
            StopBits::STOP1 => uart1.lcr.modify(|_, w| w.stop_bit().clear_bit()),
            StopBits::STOP2 => uart1.lcr.modify(|_, w| w.stop_bit().set_bit()),
        }
        match config.parity {
            Parity::ParityNone => uart1.lcr.modify(|_, w| w.par_en().clear_bit()),
            _ => uart1
                .lcr
                .modify(|_, w| unsafe { w.par_en().set_bit().par_mod().bits(config.parity as u8) }),
        }

        // baudrate = Fsys * 2 / R8_UARTx_DIV / 16 / R16_UARTx_DL
        let x = 10 * crate::sysctl::clocks().hclk.to_Hz() / 8 / config.baudrate;
        let x = ((x + 5) / 10) & 0xffff;

        uart1.div.write(|w| unsafe { w.bits(1) });
        uart1.dl.write(|w| unsafe { w.bits(x as u16) });

        // enable TX
        uart1.ier.write(|w| w.txd_en().set_bit());

        Self {}
    }

    pub fn blocking_write(&self, buf: &[u8]) {
        let uart1 = unsafe { &*pac::UART1::PTR };

        const UART_FIFO_SIZE: u8 = 8;

        for &c in buf {
            while uart1.tfc.read().tfc().bits() >= UART_FIFO_SIZE {
                // wait
            }
            uart1.thr().write(|w| unsafe { w.rbr().bits(c) });
        }
    }

    pub fn flush(&self) {
        let uart1 = unsafe { &*pac::UART1::PTR };

        while uart1.tfc.read().tfc().bits() != 0 {
            // wait
        }
    }
}

impl core::fmt::Write for Uart {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.blocking_write(s.as_bytes());
        Ok(())
    }
}
