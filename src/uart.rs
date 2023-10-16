//! UART: Uni

use core::marker::PhantomData;

use crate::dma::NoDma;
use crate::{into_ref, pac, peripherals, Peripheral, PeripheralRef};

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

#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {
    BaudrateTooLow,
    BaudrateTooHigh,
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

/// Serial error
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
    /// Buffer too large for DMA
    BufferTooLong,
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
            uart1.thr().write(|w| unsafe { w.bits(c) });
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

// ----

pub struct UartTx<'d, T: BasicInstance, TxDma = NoDma> {
    phantom: PhantomData<&'d mut T>,
    tx_dma: PeripheralRef<'d, TxDma>,
}

impl<'d, T: BasicInstance, TxDma> UartTx<'d, T, TxDma> {
    /// Useful if you only want Uart Tx. It saves 1 pin and consumes a little less power.
    pub fn new(
        peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxPin<T>> + 'd,
        tx_dma: impl Peripheral<P = TxDma> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        //T::enable();

        Self::new_inner(peri, tx, tx_dma, config)
    }

    /* pub fn new_with_cts(
        peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxPin<T>> + 'd,
        cts: impl Peripheral<P = impl CtsPin<T>> + 'd,
        tx_dma: impl Peripheral<P = TxDma> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        into_ref!(cts);

        todo!()
        Self::new_inner(peri, tx, tx_dma, config)
    }
    */

    fn new_inner(
        _peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxPin<T>> + 'd,
        tx_dma: impl Peripheral<P = TxDma> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        into_ref!(_peri, tx, tx_dma);

        // set up pin
        tx.set_as_output_with_drive_low();
        if tx.is_remap() {
            let sys = unsafe { &*pac::SYS::PTR };
            let offset = T::remap_offset();
            sys.pin_alternate
                .modify(|r, w| unsafe { w.bits(r.bits() | (0x1 << offset)) });
        }

        // set up uart
        let uart: &pac::uart0::RegisterBlock = T::regs();

        uart.fcr.write(|w| unsafe {
            w.rx_fifo_clr()
                .set_bit()
                .rx_fifo_clr()
                .set_bit()
                .fifo_en() // enable 8 byte FIFO
                .set_bit()
                .fifo_trig()
                .bits(2) // 4 bytes
        });
        uart.lcr.write(|w| unsafe { w.word_sz().bits(config.data_bits as u8) });
        match config.stop_bits {
            StopBits::STOP1 => uart.lcr.modify(|_, w| w.stop_bit().clear_bit()),
            StopBits::STOP2 => uart.lcr.modify(|_, w| w.stop_bit().set_bit()),
        }
        match config.parity {
            Parity::ParityNone => uart.lcr.modify(|_, w| w.par_en().clear_bit()),
            _ => uart
                .lcr
                .modify(|_, w| unsafe { w.par_en().set_bit().par_mod().bits(config.parity as u8) }),
        }

        // baudrate = Fsys * 2 / R8_UARTx_DIV / 16 / R16_UARTx_DL
        let x = 10 * crate::sysctl::clocks().hclk.to_Hz() / 8 / config.baudrate;
        let x = ((x + 5) / 10) & 0xffff;

        uart.div.write(|w| unsafe { w.bits(1) });
        uart.dl.write(|w| unsafe { w.bits(x as u16) });

        // enable TX
        uart.ier.write(|w| w.txd_en().set_bit());

        //tx.set_as_af(tx.af_num(), AFType::OutputPushPull);
        // configure(r, &config, T::frequency(), T::KIND, false, true)?;

        // create state once!
        //let _s = T::state();

        Ok(Self {
            tx_dma,
            phantom: PhantomData,
        })
    }

    // todo: reconfigure support

    // pub async fn write(&mut self, buffer: &[u8]) -> Result<(), Error>
    // where
    //     TxDma: crate::usart::TxDma<T>,

    pub fn blocking_write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        let uart = T::regs();

        const UART_FIFO_SIZE: u8 = 8;

        for &c in buffer {
            while uart.tfc.read().tfc().bits() >= UART_FIFO_SIZE {
                // wait
            }
            uart.thr().write(|w| unsafe { w.bits(c) });
        }

        Ok(())
    }

    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        let uart = T::regs();
        while uart.tfc.read().tfc().bits() != 0 {
            // wait
        }
        Ok(())
    }
}

// embedded-hal

impl<'d, T: BasicInstance, TxDma> core::fmt::Write for UartTx<'d, T, TxDma> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.blocking_write(s.as_bytes()).unwrap();
        Ok(())
    }
}

mod eh1 {
    use super::*;

    impl embedded_hal_nb::serial::Error for Error {
        fn kind(&self) -> embedded_hal_nb::serial::ErrorKind {
            match *self {
                Self::Framing => embedded_hal_nb::serial::ErrorKind::FrameFormat,
                Self::Noise => embedded_hal_nb::serial::ErrorKind::Noise,
                Self::Overrun => embedded_hal_nb::serial::ErrorKind::Overrun,
                Self::Parity => embedded_hal_nb::serial::ErrorKind::Parity,
                Self::BufferTooLong => embedded_hal_nb::serial::ErrorKind::Other,
            }
        }
    }

    impl<'d, T: BasicInstance, TxDma> embedded_hal_nb::serial::ErrorType for UartTx<'d, T, TxDma> {
        type Error = Error;
    }

    impl<'d, T: BasicInstance, TxDma> embedded_hal_nb::serial::Write for UartTx<'d, T, TxDma> {
        fn write(&mut self, char: u8) -> nb::Result<(), Self::Error> {
            self.blocking_write(&[char]).map_err(nb::Error::Other)
        }

        fn flush(&mut self) -> nb::Result<(), Self::Error> {
            self.blocking_flush().map_err(nb::Error::Other)
        }
    }
}

// sealed

pub(crate) mod sealed {
    // use embassy_sync::waitqueue::AtomicWaker;

    use super::*;
    use crate::interrupt;

    #[derive(Clone, Copy, PartialEq, Eq)]
    pub enum Kind {
        Uart,
        // UART with CTS, DSR, RI, DCD, DTR, RTS
        UartWithModem,
    }

    /*
    pub struct State {
        pub rx_waker: AtomicWaker,
        pub tx_waker: AtomicWaker,
    }

    impl State {
        pub const fn new() -> Self {
            Self {
                rx_waker: AtomicWaker::new(),
                tx_waker: AtomicWaker::new(),
            }
        }
    }
    */

    pub trait BasicInstance {
        const KIND: Kind;
        type Interrupt: interrupt::Interrupt;

        fn regs() -> &'static pac::uart0::RegisterBlock;
        // fn state() -> &'static ();
        fn remap_offset() -> u8;
    }

    pub trait FullInstance: BasicInstance {}
}
pub trait BasicInstance: Peripheral<P = Self> + sealed::BasicInstance + 'static + Send {}

pub trait FullInstance: sealed::FullInstance {}

// pin traits

macro_rules! pin_trait {
    ($signal:ident, $instance:path) => {
        pub trait $signal<T: $instance>: crate::gpio::Pin {
            // value for R16_PIN_ALTERNATE
            fn is_remap(&self) -> bool;
        }
    };
}

pin_trait!(RxPin, BasicInstance);
pin_trait!(TxPin, BasicInstance);

// uart peripheral traits

use self::sealed::Kind;

// ident-name, irq-name, kind?
macro_rules! impl_uart {
    ($inst:ident, $irq:ident, $kind:expr, $bit:expr) => {
        impl sealed::BasicInstance for crate::peripherals::$inst {
            const KIND: Kind = $kind;
            type Interrupt = crate::interrupt::$irq;

            fn regs() -> &'static crate::pac::uart0::RegisterBlock {
                unsafe { &*crate::pac::$inst::PTR }
            }

            /// Remap offset in R16_PIN_ALTERNATE
            fn remap_offset() -> u8 {
                $bit
            }
        }

        impl BasicInstance for peripherals::$inst {}
    };
}

impl_uart!(UART0, UART0, Kind::Uart, 4); // FIXME: UartWithModem
impl_uart!(UART1, UART1, Kind::Uart, 5);
impl_uart!(UART2, UART2, Kind::Uart, 6);
impl_uart!(UART3, UART3, Kind::Uart, 7);
