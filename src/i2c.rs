// I2C, 100KHz and 400KHz
#![macro_use]

use core::marker::PhantomData;

use fugit::HertzU32 as Hertz;

use crate::traits::pin_trait;
use crate::{interrupt, into_ref, peripherals, Peripheral};

#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    Bus,
    // Arbitration lost
    Arbitration,
    // No ack
    Nack,
    Timeout,
    // packet crc8 error
    Crc,
    Overrun,
    ZeroLengthTransfer,
}

pub(crate) mod sealed {
    use super::*;
    pub trait Instance {
        fn regs() -> &'static crate::pac::i2c::RegisterBlock;
        // fn state() -> &'static State;
    }
}

pub trait Instance: sealed::Instance + 'static {
    type Interrupt: interrupt::Interrupt;
}

pin_trait!(SclPin, Instance);
pin_trait!(SdaPin, Instance);

impl sealed::Instance for peripherals::I2C {
    fn regs() -> &'static crate::pac::i2c::RegisterBlock {
        unsafe { &*crate::pac::I2C::PTR }
    }

    // fn state() -> &'static State {
    //    static STATE: State = State::new();
    //    &STATE
    // }
}

impl Instance for peripherals::I2C {
    type Interrupt = crate::interrupt::I2C;
}

#[non_exhaustive]
#[derive(Copy, Clone)]
pub struct Config {
    pub sda_pullup: bool,
    pub scl_pullup: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            sda_pullup: false,
            scl_pullup: false,
        }
    }
}

pub struct I2c<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
}

impl<'d, T: Instance> I2c<'d, T> {
    pub fn new(
        peri: impl Peripheral<P = T> + 'd,
        scl: impl Peripheral<P = impl SclPin<T>> + 'd,
        sda: impl Peripheral<P = impl SdaPin<T>> + 'd,
        _irq: (),
        freq: Hertz,
        config: Config,
    ) -> Self {
        into_ref!(peri, scl, sda);

        let rb = T::regs();

        //T::enable();
        //T::reset();

        //T::Interrupt::unpend();
        //unsafe { T::Interrupt::enable() };

        Self { phantom: PhantomData }
    }
}

// TODO
