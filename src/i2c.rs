// I2C
#![macro_use]

use core::marker::PhantomData;

use crate::interrupt;
use crate::peripherals;
use crate::traits::pin_trait;

#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    Bus,
    Arbitration,
    Nack,
    Timeout,
    Crc,
    Overrun,
    ZeroLengthTransfer,
}

pub(crate) mod sealed {
    use super::*;
    pub trait Instance {
        fn regs() -> crate::pac::I2C;
        // fn state() -> &'static State;
    }
}

pub trait Instance: sealed::Instance + 'static {
    type Interrupt: interrupt::Interrupt;
}


pin_trait!(SclPin, Instance);
pin_trait!(SdaPin, Instance);



pub struct I2c<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
}

