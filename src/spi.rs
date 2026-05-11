//! SPI, Serial Peripheral Interface
//!
//! CH583 has SPI0 and SPI1, CH582/CH581 has SPI0 only.
//! SPI0 supports DMA, SPI1 does not.
//! SPI0 supports both master and slave mode.
//! The max clock speed is hclk/2.

pub use embedded_hal_02::spi::{Mode, Polarity, MODE_0, MODE_3};

use crate::gpio::{AnyPin, OutputDrive, Pull};
use crate::prelude::Hertz;
use crate::{into_ref, peripherals, Peripheral, PeripheralRef};

const SPI_FIFO_SIZE: u8 = 8;

#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    Framing,
    Crc,
    ModeFault,
    Overrun,
}

#[derive(Copy, Clone)]
pub enum BitOrder {
    LsbFirst,
    MsbFirst,
}

#[non_exhaustive]
#[derive(Copy, Clone)]
pub struct Config {
    // No phrase support
    /// Clock polarity
    pub clock_polarity: Polarity,
    pub bit_order: BitOrder,
    pub frequency: Hertz,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            clock_polarity: Polarity::IdleLow,
            bit_order: BitOrder::MsbFirst,
            frequency: Hertz::from_raw(1_000_000),
        }
    }
}

// TODO
#[allow(unused)]
pub struct Spi<'d, T: Instance> {
    _peri: PeripheralRef<'d, T>,
    sck: Option<PeripheralRef<'d, AnyPin>>,
    mosi: Option<PeripheralRef<'d, AnyPin>>,
    miso: Option<PeripheralRef<'d, AnyPin>>,
}

impl<'d, T: Instance> Spi<'d, T> {
    pub fn new<const REMAP: bool>(
        peri: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = impl SckPin<T, REMAP>> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T, REMAP>> + 'd,
        miso: impl Peripheral<P = impl MisoPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(sck, mosi, miso);

        if config.clock_polarity == Polarity::IdleLow {
            sck.set_low();
        } else {
            sck.set_high();
        }
        sck.set_as_output(OutputDrive::_5mA);
        mosi.set_as_output(OutputDrive::_5mA);
        miso.set_as_input(Pull::None);

        if REMAP {
            T::set_remap();
        }

        Self::new_inner(
            peri,
            Some(sck.map_into()),
            Some(mosi.map_into()),
            Some(miso.map_into()),
            config,
        )
    }

    pub fn new_rxonly<const REMAP: bool>(
        peri: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = impl SckPin<T, REMAP>> + 'd,
        miso: impl Peripheral<P = impl MisoPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(sck, miso);

        if config.clock_polarity == Polarity::IdleLow {
            sck.set_low();
        } else {
            sck.set_high();
        }
        sck.set_as_output(OutputDrive::_5mA);
        miso.set_as_input(Pull::None);

        if REMAP {
            T::set_remap();
        }

        Self::new_inner(peri, Some(sck.map_into()), None, Some(miso.map_into()), config)
    }

    pub fn new_txonly<const REMAP: bool>(
        peri: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = impl SckPin<T, REMAP>> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(sck, mosi);

        // GPIO_ModeOut_PP_5mA
        if config.clock_polarity == Polarity::IdleLow {
            sck.set_low();
        } else {
            sck.set_high();
        }
        sck.set_as_output(OutputDrive::_5mA);
        mosi.set_as_output(OutputDrive::_5mA);

        if REMAP {
            T::set_remap();
        }

        Self::new_inner(peri, Some(sck.map_into()), Some(mosi.map_into()), None, config)
    }
    pub fn new_txonly_nosck<const REMAP: bool>(
        peri: impl Peripheral<P = T> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(mosi);

        mosi.set_as_output(OutputDrive::_5mA);

        if REMAP {
            T::set_remap();
        }

        Self::new_inner(peri, None, Some(mosi.map_into()), None, config)
    }

    fn new_inner(
        peri: impl Peripheral<P = T> + 'd,
        sck: Option<PeripheralRef<'d, AnyPin>>,
        mosi: Option<PeripheralRef<'d, AnyPin>>,
        miso: Option<PeripheralRef<'d, AnyPin>>,
        config: Config,
    ) -> Self {
        into_ref!(peri);

        // TODO
        //    if false {
        //      let sys = unsafe { &*crate::pac::SYS::PTR };
        //     sys.slp_clk_off1.modify(|_, w| w.slp_clk_spi0().clear_bit());
        //}

        // set clock div
        let sysclk = crate::sysctl::clocks().hclk.to_Hz();
        let fdiv = sysclk / config.frequency.to_Hz();
        let fdiv = fdiv.min(0xff).max(2) as u8;
        if fdiv == 2 {
            // master input delay enable, for high clock speed
            T::regs().ctrl_cfg().modify(|_, w| w.mst_dly_en().set_bit());
        }
        T::regs().clock_div().write(|w| unsafe { w.clock_div().bits(fdiv) });

        // FIFO/Counter/IF clear
        T::regs().ctrl_mod().write(|w| w.all_clear().set_bit());

        // enable output
        T::regs().ctrl_mod().write(|w| {
            // UNDOCUMENTED: ALL_CLEAR must be cleared when setting OE
            w.all_clear()
                .clear_bit()
                .mosi_oe()
                .bit(mosi.is_some())
                .miso_oe()
                .bit(miso.is_some())
                .sck_oe()
                .bit(sck.is_some())
        });

        T::regs()
            .ctrl_cfg()
            .modify(|_, w| w.auto_if().set_bit().dma_enable().clear_bit());

        // mode 0 or mode 3
        match config.clock_polarity {
            // MODE_0
            Polarity::IdleLow => T::regs().ctrl_mod().modify(|_, w| w.mst_sck_mod().clear_bit()), // default
            // MODE_3
            Polarity::IdleHigh => T::regs().ctrl_mod().modify(|_, w| w.mst_sck_mod().set_bit()),
        };
        match config.bit_order {
            BitOrder::MsbFirst => T::regs().ctrl_cfg().modify(|_, w| w.bit_order().clear_bit()), // default
            BitOrder::LsbFirst => T::regs().ctrl_cfg().modify(|_, w| w.bit_order().set_bit()),
        };

        Self {
            _peri: peri,
            sck,
            mosi,
            miso,
        }
    }

    // transfer fn

    pub fn blocking_write_byte(&mut self, byte: u8) -> Result<(), Error> {
        let rb = T::regs();
        rb.ctrl_mod().modify(|_, w| w.fifo_dir().clear_bit());
        rb.buffer().write(|w| unsafe { w.bits(byte) });
        while !rb.int_flag().read().free().bit_is_set() {}
        Ok(())
    }

    pub fn blocking_read_byte(&mut self) -> Result<u8, Error> {
        let rb = T::regs();
        rb.ctrl_mod().modify(|_, w| w.fifo_dir().clear_bit()); // ?? in EVT
        rb.buffer().write(|w| unsafe { w.bits(0xFF) });
        while !rb.int_flag().read().free().bit_is_set() {}
        Ok(rb.buffer().read().bits())
    }

    pub fn blocking_write(&mut self, words: &[u8]) -> Result<(), Error> {
        if words.len() > 4095 {
            return Err(Error::Overrun);
        }

        let rb = T::regs();
        // set fifo direction to output
        rb.ctrl_mod().modify(|_, w| w.fifo_dir().clear_bit());

        rb.total_cnt()
            .write(|w| unsafe { w.total_cnt().bits(words.len() as _) });

        rb.int_flag().write(|w| w.if_cnt_end().set_bit()); // end CNT set

        for &byte in words {
            while rb.fifo_count().read().bits() >= SPI_FIFO_SIZE {}
            rb.fifo().write(|w| unsafe { w.fifo().bits(byte) });
        }

        while rb.fifo_count().read().bits() != 0 {}

        Ok(())
    }

    pub fn blocking_read(&mut self, words: &mut [u8]) -> Result<(), Error> {
        T::regs().ctrl_mod().modify(|_, w| w.fifo_dir().set_bit());

        let read_len = words.len();
        if read_len > 4095 {
            return Err(Error::Overrun);
        }

        T::regs()
            .total_cnt()
            .write(|w| unsafe { w.total_cnt().bits(words.len() as _) });

        T::regs().int_flag().write(|w| w.if_cnt_end().set_bit()); // end CNT set

        for i in 0..read_len {
            if T::regs().fifo_count().read().bits() > 0 {
                words[i] = T::regs().fifo().read().bits();
            }
        }

        Ok(())
    }
}

mod eh02 {
    use super::*;

    impl<'d, T: Instance> embedded_hal_02::blocking::spi::Write<u8> for Spi<'d, T> {
        type Error = Error;

        fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
            self.blocking_write(words)
        }
    }

    impl<'d, T: Instance> embedded_hal_02::blocking::spi::Transfer<u8> for Spi<'d, T> {
        type Error = Error;

        fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
            self.blocking_write(words)?;
            self.blocking_read(words)?;

            Ok(words)
        }
    }
}

// - instance trait

pub(crate) mod sealed {
    pub trait Instance {
        fn regs() -> &'static crate::pac::spi0::RegisterBlock;

        fn set_remap();
    }
}

pub trait Instance: Peripheral<P = Self> + sealed::Instance {}

impl sealed::Instance for peripherals::SPI0 {
    fn regs() -> &'static crate::pac::spi0::RegisterBlock {
        unsafe { &*crate::pac::SPI0::PTR }
    }

    fn set_remap() {
        let gpioctl = unsafe { &*crate::pac::GPIOCTL::PTR };
        gpioctl.pin_alternate().modify(|_, w| w.spi0().set_bit());
    }
}
impl Instance for peripherals::SPI0 {}
// All pins require REMAP to be the same
pub trait CsPin<T: Instance, const REMAP: bool>: crate::gpio::Pin {}
pub trait SckPin<T: Instance, const REMAP: bool>: crate::gpio::Pin {}
pub trait MosiPin<T: Instance, const REMAP: bool>: crate::gpio::Pin {}
pub trait MisoPin<T: Instance, const REMAP: bool>: crate::gpio::Pin {}

// - Pin config

macro_rules! impl_pin {
    ($pin:ident, $instance:ident, $function:ident, $remap:expr) => {
        impl $function<peripherals::$instance, $remap> for peripherals::$pin {}
    };
}

impl_pin!(PA12, SPI0, CsPin, false);
impl_pin!(PA13, SPI0, SckPin, false);
impl_pin!(PA14, SPI0, MosiPin, false);
impl_pin!(PA15, SPI0, MisoPin, false);

impl_pin!(PB12, SPI0, CsPin, true);
impl_pin!(PB13, SPI0, SckPin, true);
impl_pin!(PB14, SPI0, MosiPin, true);
impl_pin!(PB15, SPI0, MisoPin, true);

// only available on CH583, no CS, no REMAP
//impl_pin!(PA0, SPI1, SckPin, false);
//impl_pin!(PA1, SPI1, MosiPin, false);
//impl_pin!(PA2, SPI1, MisoPin, false);
