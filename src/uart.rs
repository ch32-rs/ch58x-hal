//! UART, Universal Asynchronous Receiver/Transmitter

use core::marker::PhantomData;

use crate::gpio::{OutputDrive, Pull};
use crate::{into_ref, pac, peripherals, Peripheral};

const UART_FIFO_SIZE: u8 = 8;

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

// ----

pub struct UartTx<'d, T: BasicInstance> {
    phantom: PhantomData<&'d mut T>,
}

impl<'d, T: BasicInstance> UartTx<'d, T> {
    /// Useful if you only want Uart Tx. It saves 1 pin and consumes a little less power.
    pub fn new<const REMAP: bool>(
        peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        //T::enable();

        Self::new_inner(peri, tx, config)
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

    fn new_inner<const REMAP: bool>(
        _peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        into_ref!(_peri, tx);

        // set up pin
        tx.set_as_output(OutputDrive::_5mA);
        if REMAP {
            T::set_remap();
        }

        let rb = T::regs();
        configure(rb, &config, true, false)?;

        // create state once!
        //let _s = T::state();

        Ok(Self { phantom: PhantomData })
    }

    // todo: reconfigure support

    // pub async fn write(&mut self, buffer: &[u8]) -> Result<(), Error>
    // where
    //     TxDma: crate::usart::TxDma<T>,

    pub fn blocking_write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        let rb = T::regs();

        for &c in buffer {
            // wait
            while rb.tfc().read().bits() >= UART_FIFO_SIZE {}
            rb.thr().write(|w| unsafe { w.bits(c) });
        }
        Ok(())
    }

    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        let rb = T::regs();

        while rb.tfc().read().bits() != 0 {}
        Ok(())
    }
}

pub struct UartRx<'d, T: BasicInstance> {
    phantom: PhantomData<&'d mut T>,
}

impl<'d, T: BasicInstance> UartRx<'d, T> {
    /// Useful if you only want Uart Rx. It saves 1 pin and consumes a little less power.
    pub fn new<const REMAP: bool>(
        peri: impl Peripheral<P = T> + 'd,
        // _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        rx: impl Peripheral<P = impl RxPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        // T::enable();

        Self::new_inner(peri, rx, config)
    }

    fn new_inner<const REMAP: bool>(
        peri: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = impl RxPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        use crate::interrupt::Interrupt;
        into_ref!(peri, rx);
        let _ = peri;

        rx.set_as_input(Pull::Up);
        if REMAP {
            T::set_remap();
        }

        let rb = T::regs();
        configure(rb, &config, false, true)?;

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        // create state once!
        //let _s = T::state();

        Ok(Self { phantom: PhantomData })
    }

    fn check_rx_flags(&self) -> Result<bool, Error> {
        let rb = T::regs();
        let lsr = rb.lsr().read();
        if lsr.err_rx_fifo().bit() {
            if lsr.frame_err().bit() {
                return Err(Error::Framing);
            } else if lsr.par_err().bit() {
                return Err(Error::Parity);
            } else if lsr.over_err().bit() {
                return Err(Error::Overrun);
            }
        }
        Ok(lsr.data_rdy().bit())
    }

    // fifo disabled
    /* pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        let rb = T::regs();
        for b in buffer {
            while !self.check_rx_flags()? {}
            unsafe { *b = rb.rbr().read().bits() as u8 };
        }
        Ok(())
    } */

    // fifo enabled
    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        let rb = T::regs();
        for b in buffer {
            while !self.check_rx_flags()? {}
            if rb.rfc().read().bits() > 0 {
                *b = rb.rbr().read().bits() as u8;
            }
        }
        Ok(())
    }

    pub fn nb_read(&mut self) -> Result<u8, nb::Error<Error>> {
        let rb = T::regs();
        if self.check_rx_flags()? {
            Ok(rb.rbr().read().bits() as u8)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

pub struct Uart<'d, T: BasicInstance> {
    tx: UartTx<'d, T>,
    rx: UartRx<'d, T>,
}

impl<'d, T: BasicInstance> Uart<'d, T> {
    pub fn new<const REMAP: bool>(
        peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        rx: impl Peripheral<P = impl RxPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        // T::enable();

        Self::new_inner(peri, tx, rx, config)
    }

    fn new_inner<const REMAP: bool>(
        peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        rx: impl Peripheral<P = impl RxPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        use crate::interrupt::Interrupt;
        into_ref!(peri, tx, rx);
        let _ = peri;

        tx.set_as_output(OutputDrive::_5mA);
        rx.set_as_input(Pull::Up);

        if REMAP {
            T::set_remap();
        }

        let rb = T::regs();
        configure(rb, &config, true, true)?;

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        // create state once!
        //let _s = T::state();

        Ok(Self {
            tx: UartTx { phantom: PhantomData },
            rx: UartRx { phantom: PhantomData },
        })
    }

    pub fn blocking_write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        self.tx.blocking_write(buffer)
    }

    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        self.tx.blocking_flush()
    }

    pub fn nb_read(&mut self) -> Result<u8, nb::Error<Error>> {
        self.rx.nb_read()
    }

    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        self.rx.blocking_read(buffer)
    }

    /// Split the Uart into a transmitter and receiver, which is
    /// particularly useful when having two tasks correlating to
    /// transmitting and receiving.
    pub fn split(self) -> (UartTx<'d, T>, UartRx<'d, T>) {
        (self.tx, self.rx)
    }
}

impl<'d, T: BasicInstance> core::fmt::Write for UartTx<'d, T> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.blocking_write(s.as_bytes()).unwrap();
        Ok(())
    }
}

fn configure(
    rb: &pac::uart0::RegisterBlock,
    config: &Config,
    enable_tx: bool,
    enable_rx: bool,
) -> Result<(), ConfigError> {
    if !enable_rx && !enable_tx {
        panic!("USART: At least one of RX or TX should be enabled");
    }

    rb.fcr().write(|w| {
        unsafe {
            w.rx_fifo_clr()
                .set_bit()
                .tx_fifo_clr()
                .set_bit()
                .fifo_en() // enable 8 byte FIFO
                .set_bit()
                .fifo_trig()
                .bits(0b00) // 1 bytes to send
        }
    });
    rb.lcr().write(|w| unsafe { w.word_sz().bits(config.data_bits as u8) });
    match config.stop_bits {
        StopBits::STOP1 => rb.lcr().modify(|_, w| w.stop_bit().clear_bit()),
        StopBits::STOP2 => rb.lcr().modify(|_, w| w.stop_bit().set_bit()),
    };
    match config.parity {
        Parity::ParityNone => rb.lcr().modify(|_, w| w.par_en().clear_bit()),
        _ => rb
            .lcr()
            .modify(|_, w| unsafe { w.par_en().set_bit().par_mod().bits(config.parity as u8) }),
    };

    // baudrate = Fsys * 2 / R8_UARTx_DIV / 16 / R16_UARTx_DL
    // match some common baudrates
    let (div, dl) = match (crate::sysctl::clocks().hclk.to_Hz(), config.baudrate) {
        (60_000_000, 115200) => (13, 5),
        (60_000_000, 8600) => (8, 109),
        _ => {
            let x = 10 * crate::sysctl::clocks().hclk.to_Hz() / 8 / config.baudrate;
            let x = ((x + 5) / 10) & 0xffff;

            (1, x as u16)
        }
    };

    rb.div().write(|w| unsafe { w.bits(div) });
    rb.dl().write(|w| unsafe { w.bits(dl) });

    // enable TX
    if enable_tx {
        rb.ier().modify(|_, w| w.txd_en().bit(true));
    }

    Ok(())
}

// embedded-hal
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

    impl<'d, T: BasicInstance> embedded_hal_nb::serial::ErrorType for UartTx<'d, T> {
        type Error = Error;
    }

    impl<'d, T: BasicInstance> embedded_hal_nb::serial::Write for UartTx<'d, T> {
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
        type Interrupt: interrupt::Interrupt;

        fn regs() -> &'static pac::uart0::RegisterBlock;
        // fn state() -> &'static ();
        fn set_remap();
    }

    pub trait FullInstance: BasicInstance {}
}
pub trait BasicInstance: Peripheral<P = Self> + sealed::BasicInstance + 'static + Send {}

// UART with CTS, DSR, RI, DCD, DTR, RTS
pub trait FullInstance: sealed::FullInstance {}

// uart peripheral traits

// ident-name, irq-name, kind?
macro_rules! impl_uart {
    ($inst:ident, $irq:ident, $remap_field:ident) => {
        impl sealed::BasicInstance for crate::peripherals::$inst {
            type Interrupt = crate::interrupt::$irq;

            fn regs() -> &'static crate::pac::uart0::RegisterBlock {
                unsafe { &*crate::pac::$inst::PTR }
            }

            /// Remap offset in R16_PIN_ALTERNATE
            fn set_remap() {
                let gpioctl = unsafe { &*pac::GPIOCTL::PTR };
                gpioctl.pin_alternate().modify(|_, w| w.$remap_field().set_bit());
            }
        }

        impl BasicInstance for peripherals::$inst {}
    };
}

impl_uart!(UART0, UART0, uart0); // FIXME: UartWithModem, a FullInstance
impl_uart!(UART1, UART1, uart1);
impl_uart!(UART2, UART2, uart2);
impl_uart!(UART3, UART3, uart3);

// pin traits

macro_rules! pin_trait {
    ($signal:ident, $instance:path) => {
        pub trait $signal<T: $instance, const REMAP: bool>: crate::gpio::Pin {}
    };
}

pin_trait!(RxPin, BasicInstance);
pin_trait!(TxPin, BasicInstance);

macro_rules! pin_trait_impl {
    (crate::$mod:ident::$trait:ident, $instance:ident, $pin:ident, $remap:expr) => {
        impl crate::$mod::$trait<crate::peripherals::$instance, $remap> for crate::peripherals::$pin {}
    };
}

pin_trait_impl!(crate::uart::TxPin, UART0, PB7, false);
pin_trait_impl!(crate::uart::RxPin, UART0, PB4, false);
pin_trait_impl!(crate::uart::TxPin, UART0, PA14, true);
pin_trait_impl!(crate::uart::RxPin, UART0, PA15, true);

pin_trait_impl!(crate::uart::TxPin, UART1, PA9, false);
pin_trait_impl!(crate::uart::RxPin, UART1, PA8, false);
pin_trait_impl!(crate::uart::TxPin, UART1, PB13, true);
pin_trait_impl!(crate::uart::RxPin, UART1, PB12, true);

pin_trait_impl!(crate::uart::TxPin, UART2, PA7, false);
pin_trait_impl!(crate::uart::RxPin, UART2, PA6, false);
pin_trait_impl!(crate::uart::TxPin, UART2, PB23, true);
pin_trait_impl!(crate::uart::RxPin, UART2, PB22, true);

pin_trait_impl!(crate::uart::TxPin, UART3, PA5, false);
pin_trait_impl!(crate::uart::RxPin, UART3, PA4, false);
pin_trait_impl!(crate::uart::TxPin, UART3, PB21, true);
pin_trait_impl!(crate::uart::RxPin, UART3, PB20, true);
