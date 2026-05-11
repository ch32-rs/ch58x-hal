//! I2C, 100KHz and 400KHz, no DMA support
//!
//! This implementation is based on STM32's v1 i2c in the embassy project.
#![macro_use]

use core::marker::PhantomData;

use fugit::HertzU32 as Hertz;

use crate::gpio::Pull;
use crate::{interrupt, into_ref, peripherals, Peripheral};

// Any of BERR=1；ARLO=1；AF=1；OVR=1；PECERR=1； TIMEOUT=1；SMBAlert=1。
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Bus error, BERR
    Bus,
    // Arbitration lost
    Arbitration,
    // No ack, AF
    Nack,
    Timeout,
    // packet crc8 error, PEC
    Crc,
    // OVR
    Overrun,
    ZeroLengthTransfer,
}

pub(crate) mod sealed {
    pub trait Instance {
        fn regs() -> &'static crate::pac::i2c::RegisterBlock;
        // fn state() -> &'static State;
    }
}

pub trait Instance: sealed::Instance + 'static {
    type Interrupt: interrupt::Interrupt;
}

// REMAP is required to be the same for both pins
pub trait SclPin<T: Instance, const REMAP: bool>: crate::gpio::Pin {}
pub trait SdaPin<T: Instance, const REMAP: bool>: crate::gpio::Pin {}

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

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Duty {
    Duty2_1 = 0,
    Duty16_9 = 1,
}

#[non_exhaustive]
#[derive(Copy, Clone)]
pub struct Config {
    pub sda_pullup: bool,
    pub scl_pullup: bool,
    pub frequency: Hertz,
    pub duty: Duty,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            sda_pullup: false,
            scl_pullup: false,
            frequency: Hertz::from_raw(100_000), // default slow
            duty: Duty::Duty2_1,
        }
    }
}

pub struct I2c<'d, T: Instance> {
    phantom: PhantomData<(&'d mut T,)>,
}

impl<'d, T: Instance> I2c<'d, T> {
    pub fn new<const REMAP: bool>(
        peri: impl Peripheral<P = T> + 'd,
        scl: impl Peripheral<P = impl SclPin<T, REMAP>> + 'd,
        sda: impl Peripheral<P = impl SdaPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(peri, scl, sda);
        let _ = peri;

        if REMAP {
            let gpioctl = unsafe { &*crate::pac::GPIOCTL::PTR };
            gpioctl.pin_alternate().modify(|_, w| w.i2c().set_bit());
        }
        let scl_pull = if config.scl_pullup { Pull::Up } else { Pull::None };
        let sda_pull = if config.sda_pullup { Pull::Up } else { Pull::None };
        scl.set_as_input(scl_pull);
        sda.set_as_input(sda_pull);

        let rb = T::regs();

        // TODO: enable peripheral

        // reset peripheral
        rb.ctrl1().modify(|_, w| w.swrst().set_bit());
        rb.ctrl1().modify(|_, w| w.swrst().clear_bit());

        // 60MHz is the max frequency
        let sysclk = crate::sysctl::clocks().hclk.to_Hz();
        let sysclk_mhz = crate::sysctl::clocks().hclk.to_MHz();
        let i2c_clk = config.frequency.to_Hz();

        rb.ctrl2()
            .modify(|_, w| unsafe { w.freq().bits((sysclk / 1_000_000) as u8) });

        rb.ctrl1().modify(|_, w| w.pe().clear_bit());

        if config.frequency.to_Hz() <= 100_000 {
            let tmp = (sysclk / (i2c_clk * 2)) & 0x0FFF;
            let tmp = u32::max(tmp, 0x04);

            let val = u32::min(sysclk_mhz + 1, 0x3F);

            rb.rtr().write(|w| unsafe { w.trise().bits(val as _) });
            rb.ckcfgr().write(|w| unsafe { w.ccr().bits(tmp as _) });
        } else {
            // high speed, use duty cycle
            let tmp = if config.duty == Duty::Duty2_1 {
                (sysclk / (i2c_clk * 3)) & 0x0FFF
            } else {
                (sysclk / (i2c_clk * 25)) & 0x0FFF
            };
            let tmp = u32::max(tmp, 0x01);

            let val = (sysclk_mhz * 300) / 1000 + 1;

            rb.rtr().write(|w| unsafe { w.trise().bits(val as _) });
            rb.ckcfgr().write(|w| unsafe {
                w.f_s()
                    .set_bit()
                    .duty()
                    .variant(config.duty as u8 != 0)
                    .ccr()
                    .bits(tmp as u16)
            });
        }

        rb.ctrl1().modify(|_, w| w.pe().set_bit());

        // i2c type, ACK=master mode
        rb.ctrl1()
            .modify(|_, w| w.smbus().clear_bit().smbtype().clear_bit().ack().clear_bit());

        Self { phantom: PhantomData }
    }

    fn check_and_clear_error_flags(&self) -> Result<crate::pac::i2c::star1::R, Error> {
        // Note that flags should only be cleared once they have been registered. If flags are
        // cleared otherwise, there may be an inherent race condition and flags may be missed.
        let star1 = T::regs().star1().read();

        if star1.timeout().bit() {
            T::regs().star1().modify(|_, w| w.timeout().clear_bit());
            return Err(Error::Timeout);
        }

        if star1.pecerr().bit() {
            T::regs().star1().modify(|_, w| w.pecerr().clear_bit());
            return Err(Error::Crc);
        }

        if star1.ovr().bit() {
            T::regs().star1().modify(|_, w| w.ovr().clear_bit());
            return Err(Error::Overrun);
        }

        if star1.af().bit() {
            T::regs().star1().modify(|_, w| w.af().clear_bit());
            return Err(Error::Nack);
        }

        if star1.arlo().bit() {
            T::regs().star1().modify(|_, w| w.arlo().clear_bit());
            return Err(Error::Arbitration);
        }

        // The errata indicates that BERR may be incorrectly detected. It recommends ignoring and
        // clearing the BERR bit instead.
        if star1.berr().bit() {
            T::regs().star1().modify(|_, w| w.berr().clear_bit());
        }

        Ok(star1)
    }

    /// STAR1 and STAR2 have a complex read-clear rule. So we need to read STAR1 first.
    fn write_bytes(
        &mut self,
        addr: u8,
        bytes: &[u8],
        check_timeout: impl Fn() -> Result<(), Error>,
    ) -> Result<(), Error> {
        // Send a START condition
        let rb = T::regs();

        rb.ctrl1().modify(|_, w| w.start().set_bit());

        // Wait until START condition was generated
        while !self.check_and_clear_error_flags()?.sb().bit() {
            check_timeout()?;
        }

        // Also wait until signalled we're master and everything is waiting for us
        while {
            self.check_and_clear_error_flags()?;

            let sr2 = rb.star2().read();
            !sr2.msl().bit() && !sr2.busy().bit()
        } {
            check_timeout()?;
        }

        // Set up current address, we're trying to talk to
        rb.datar().write(|w| unsafe { w.datar().bits(addr << 1) });

        // Wait until address was sent
        // Wait for the address to be acknowledged
        // Check for any I2C errors. If a NACK occurs, the ADDR bit will never be set.
        while !self.check_and_clear_error_flags()?.addr().bit() {
            check_timeout()?;
        }

        // Clear condition by reading SR2
        let _ = rb.star2().read();

        // Send bytes
        for c in bytes {
            self.send_byte(*c, &check_timeout)?;
        }
        // Fallthrough is success
        Ok(())
    }

    fn send_byte(&self, byte: u8, check_timeout: impl Fn() -> Result<(), Error>) -> Result<(), Error> {
        // Wait until we're ready for sending
        while {
            // Check for any I2C errors. If a NACK occurs, the ADDR bit will never be set.
            !self.check_and_clear_error_flags()?.tx_e().bit()
        } {
            check_timeout()?;
        }

        // Push out a byte of data
        T::regs().datar().write(|w| unsafe { w.datar().bits(byte) });

        // Wait until byte is transferred
        while {
            // Check for any potential error conditions.
            !self.check_and_clear_error_flags()?.btf().bit()
        } {
            check_timeout()?;
        }

        Ok(())
    }

    fn recv_byte(&self, check_timeout: impl Fn() -> Result<(), Error>) -> Result<u8, Error> {
        while {
            // Check for any potential error conditions.
            self.check_and_clear_error_flags()?;

            !T::regs().star1().read().rx_ne().bit()
        } {
            check_timeout()?;
        }

        let value = T::regs().datar().read().datar().bits();
        Ok(value)
    }

    pub fn blocking_read_timeout(
        &mut self,
        addr: u8,
        buffer: &mut [u8],
        check_timeout: impl Fn() -> Result<(), Error>,
    ) -> Result<(), Error> {
        if let Some((last, buffer)) = buffer.split_last_mut() {
            // Send a START condition and set ACK bit
            T::regs().ctrl1().modify(|_, w| w.start().set_bit().ack().set_bit());

            // Wait until START condition was generated
            while !self.check_and_clear_error_flags()?.sb().bit() {
                check_timeout()?;
            }

            // Also wait until signalled we're master and everything is waiting for us
            while {
                let sr2 = T::regs().star2().read();
                !sr2.msl().bit() && !sr2.busy().bit()
            } {
                check_timeout()?;
            }

            // Set up current address, we're trying to talk to
            T::regs().datar().write(|w| unsafe { w.datar().bits((addr << 1) + 1) });

            // Wait until address was sent
            // Wait for the address to be acknowledged
            while !self.check_and_clear_error_flags()?.addr().bit() {
                check_timeout()?;
            }

            // Clear condition by reading SR2
            let _ = T::regs().star2().read();

            // Receive bytes into buffer
            for c in buffer {
                *c = self.recv_byte(&check_timeout)?;
            }

            // Prepare to send NACK then STOP after next byte
            T::regs().ctrl1().modify(|_, w| w.ack().clear_bit().stop().set_bit());

            // Receive last byte
            *last = self.recv_byte(&check_timeout)?;

            // Wait for the STOP to be sent.
            while T::regs().ctrl1().read().stop().bit() {
                check_timeout()?;
            }

            // Fallthrough is success
            Ok(())
        } else {
            Err(Error::Overrun)
        }
    }

    pub fn blocking_read(&mut self, addr: u8, read: &mut [u8]) -> Result<(), Error> {
        self.blocking_read_timeout(addr, read, || Ok(()))
    }

    pub fn blocking_write_timeout(
        &mut self,
        addr: u8,
        write: &[u8],
        check_timeout: impl Fn() -> Result<(), Error>,
    ) -> Result<(), Error> {
        self.write_bytes(addr, write, &check_timeout)?;
        // Send a STOP condition
        T::regs().ctrl1().modify(|_, w| w.stop().set_bit());
        // Wait for STOP condition to transmit.
        while T::regs().ctrl1().read().stop().bit() {
            check_timeout()?;
        }

        // Fallthrough is success
        Ok(())
    }

    pub fn blocking_write(&mut self, addr: u8, write: &[u8]) -> Result<(), Error> {
        self.blocking_write_timeout(addr, write, || Ok(()))
    }

    pub fn blocking_write_read_timeout(
        &mut self,
        addr: u8,
        write: &[u8],
        read: &mut [u8],
        check_timeout: impl Fn() -> Result<(), Error>,
    ) -> Result<(), Error> {
        self.write_bytes(addr, write, &check_timeout)?;
        self.blocking_read_timeout(addr, read, &check_timeout)?;

        Ok(())
    }

    pub fn blocking_write_read(&mut self, addr: u8, write: &[u8], read: &mut [u8]) -> Result<(), Error> {
        self.blocking_write_read_timeout(addr, write, read, || Ok(()))
    }
}

impl<'d, T: Instance> Drop for I2c<'d, T> {
    fn drop(&mut self) {
        T::regs().ctrl1().modify(|_, w| w.pe().clear_bit());
    }
}

mod eh1 {
    use super::*;

    impl embedded_hal_1::i2c::Error for Error {
        fn kind(&self) -> embedded_hal_1::i2c::ErrorKind {
            match *self {
                Self::Bus => embedded_hal_1::i2c::ErrorKind::Bus,
                Self::Arbitration => embedded_hal_1::i2c::ErrorKind::ArbitrationLoss,
                Self::Nack => {
                    embedded_hal_1::i2c::ErrorKind::NoAcknowledge(embedded_hal_1::i2c::NoAcknowledgeSource::Unknown)
                }
                Self::Timeout => embedded_hal_1::i2c::ErrorKind::Other,
                Self::Crc => embedded_hal_1::i2c::ErrorKind::Other,
                Self::Overrun => embedded_hal_1::i2c::ErrorKind::Overrun,
                Self::ZeroLengthTransfer => embedded_hal_1::i2c::ErrorKind::Other,
            }
        }
    }

    impl<'d, T: Instance> embedded_hal_1::i2c::ErrorType for I2c<'d, T> {
        type Error = Error;
    }

    impl<'d, T: Instance> embedded_hal_1::i2c::I2c for I2c<'d, T> {
        fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
            self.blocking_read(address, read)
        }

        fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
            self.blocking_write(address, write)
        }

        fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
            self.blocking_write_read(address, write, read)
        }

        fn transaction(
            &mut self,
            _address: u8,
            _operations: &mut [embedded_hal_1::i2c::Operation<'_>],
        ) -> Result<(), Self::Error> {
            todo!();
        }
    }
}

// - Pin config

macro_rules! impl_pin {
    ($pin:ident, $instance:ident, $function:ident, $remap:expr) => {
        impl $function<peripherals::$instance, $remap> for peripherals::$pin {}
    };
}

impl_pin!(PB13, I2C, SclPin, false);
impl_pin!(PB12, I2C, SdaPin, false);

impl_pin!(PB21, I2C, SclPin, true);
impl_pin!(PB20, I2C, SdaPin, true);
