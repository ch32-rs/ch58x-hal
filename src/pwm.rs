use crate::peripherals::{PA12, PA13, PA6, PA7, PB0, PB1, PB14, PB2, PB23, PB3, PB4, PB6, PB7, PWMX};
use crate::{into_ref, pac, Peripheral};

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// PWM channel
pub enum Channel {
    Pwm4 = 0x01,
    Pwm5 = 0x02,
    Pwm6 = 0x04,
    Pwm7 = 0x08,
    Pwm8 = 0x10,
    Pwm9 = 0x20,
    Pwm10 = 0x40,
    Pwm11 = 0x80,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Cycle configuration, refer to datasheet
pub enum Cycle {
    Cycle256,
    Cycle255,
    Cycle128,
    Cycle127,
    Cycle64,
    Cycle63,
    Cycle32,
    Cycle31,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Polarity {
    /// Default Low, High Action
    Low,
    /// Default High, Low Action
    High,
}

/// Peripheral-wide PWM settings
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    pub clock_div: u8,
    pub cycle_sel: Cycle,
}

/// PWM HAL struct
pub struct Pwm<'d, T: Instance> {
    #[allow(unused)]
    pwm: crate::PeripheralRef<'d, T>,
}

impl<'d, T> Pwm<'d, T>
where
    T: Instance,
{
    /// Construct a new pwm hal instance
    pub fn new(pwm_inst: impl Peripheral<P = T> + 'd, config: Config) -> Self {
        into_ref!(pwm_inst);
        let mut pwm = Self { pwm: pwm_inst };
        pwm.set_config(config);
        pwm
    }

    /// Set the peripheral-wide [`Config`]
    pub fn set_config(&mut self, config: Config) {
        let rb = T::regs();
        rb.pwm_clock_div().write(|w| unsafe { w.bits(config.clock_div) });
        self.cycle_sel(config.cycle_sel);
    }

    /// Set the duty of a specific [`PwmPin`], from 0-255 as 0-100%
    pub fn set_duty(&mut self, duty: u8, pin: &mut impl PwmPin<T>) {
        let rb = T::regs();
        match pin.channel() {
            Channel::Pwm4 => rb.pwm4_data().write(|w| unsafe { w.bits(duty) }),
            Channel::Pwm5 => rb.pwm5_data().write(|w| unsafe { w.bits(duty) }),
            Channel::Pwm6 => rb.pwm6_data().write(|w| unsafe { w.bits(duty) }),
            Channel::Pwm7 => rb.pwm7_data().write(|w| unsafe { w.bits(duty) }),
            Channel::Pwm8 => rb.pwm8_data().write(|w| unsafe { w.bits(duty) }),
            Channel::Pwm9 => rb.pwm9_data().write(|w| unsafe { w.bits(duty) }),
            Channel::Pwm10 => rb.pwm10_data().write(|w| unsafe { w.bits(duty) }),
            Channel::Pwm11 => rb.pwm11_data().write(|w| unsafe { w.bits(duty) }),
        }
    }

    /// Set the polarity of a specific pin, see [`Polarity`] for more details
    pub fn set_polarity(&mut self, polarity: Polarity, pin: &mut impl PwmPin<T>) {
        let rb = T::regs();
        match pin.channel() {
            Channel::Pwm4 => rb
                .pwm_polar()
                .modify(|_, w| w.pwm4_polar().bit(polarity == Polarity::High)),
            Channel::Pwm5 => rb
                .pwm_polar()
                .modify(|_, w| w.pwm5_polar().bit(polarity == Polarity::High)),
            Channel::Pwm6 => rb
                .pwm_polar()
                .modify(|_, w| w.pwm6_polar().bit(polarity == Polarity::High)),
            Channel::Pwm7 => rb
                .pwm_polar()
                .modify(|_, w| w.pwm7_polar().bit(polarity == Polarity::High)),
            Channel::Pwm8 => rb
                .pwm_polar()
                .modify(|_, w| w.pwm8_polar().bit(polarity == Polarity::High)),
            Channel::Pwm9 => rb
                .pwm_polar()
                .modify(|_, w| w.pwm9_polar().bit(polarity == Polarity::High)),
            Channel::Pwm10 => rb
                .pwm_polar()
                .modify(|_, w| w.pwm10_polar().bit(polarity == Polarity::High)),
            Channel::Pwm11 => rb
                .pwm_polar()
                .modify(|_, w| w.pwm11_polar().bit(polarity == Polarity::High)),
        }
    }

    /// Enable the output of a specific [`PwmPin`]
    pub fn enable(&mut self, pin: &mut impl PwmPin<T>) {
        let rb = T::regs();
        match pin.channel() {
            Channel::Pwm4 => rb.pwm_out_en().modify(|_, w| w.pwm4_out_en().bit(true)),
            Channel::Pwm5 => rb.pwm_out_en().modify(|_, w| w.pwm5_out_en().bit(true)),
            Channel::Pwm6 => rb.pwm_out_en().modify(|_, w| w.pwm6_out_en().bit(true)),
            Channel::Pwm7 => rb.pwm_out_en().modify(|_, w| w.pwm7_out_en().bit(true)),
            Channel::Pwm8 => rb.pwm_out_en().modify(|_, w| w.pwm8_out_en().bit(true)),
            Channel::Pwm9 => rb.pwm_out_en().modify(|_, w| w.pwm9_out_en().bit(true)),
            Channel::Pwm10 => rb.pwm_out_en().modify(|_, w| w.pwm10_out_en().bit(true)),
            Channel::Pwm11 => rb.pwm_out_en().modify(|_, w| w.pwm11_out_en().bit(true)),
        }
    }

    /// Disable the output of a specific [`PwmPin`]
    pub fn disable(&mut self, pin: &mut impl PwmPin<T>) {
        let rb = T::regs();
        match pin.channel() {
            Channel::Pwm4 => rb.pwm_out_en().modify(|_, w| w.pwm4_out_en().bit(false)),
            Channel::Pwm5 => rb.pwm_out_en().modify(|_, w| w.pwm5_out_en().bit(false)),
            Channel::Pwm6 => rb.pwm_out_en().modify(|_, w| w.pwm6_out_en().bit(false)),
            Channel::Pwm7 => rb.pwm_out_en().modify(|_, w| w.pwm7_out_en().bit(false)),
            Channel::Pwm8 => rb.pwm_out_en().modify(|_, w| w.pwm8_out_en().bit(false)),
            Channel::Pwm9 => rb.pwm_out_en().modify(|_, w| w.pwm9_out_en().bit(false)),
            Channel::Pwm10 => rb.pwm_out_en().modify(|_, w| w.pwm10_out_en().bit(false)),
            Channel::Pwm11 => rb.pwm_out_en().modify(|_, w| w.pwm11_out_en().bit(false)),
        }
    }

    /// Set the [`Cycle`] config of the pwmx peripheral, refer to the datasheet
    fn cycle_sel(&mut self, cycle: Cycle) {
        let rb = T::regs();
        match cycle {
            Cycle::Cycle256 => rb.pwm_config().modify(|r, w| unsafe { w.bits(r.bits() & 0xF0) }),
            Cycle::Cycle255 => rb
                .pwm_config()
                .modify(|r, w| unsafe { w.bits((r.bits() & 0xF0) | 0x01) }),
            Cycle::Cycle128 => rb
                .pwm_config()
                .modify(|r, w| unsafe { w.bits((r.bits() & 0xF0) | (1 << 2)) }),
            Cycle::Cycle127 => rb
                .pwm_config()
                .modify(|r, w| unsafe { w.bits((r.bits() & 0xF0) | (1 << 2) | 0x01) }),
            Cycle::Cycle64 => rb
                .pwm_config()
                .modify(|r, w| unsafe { w.bits((r.bits() & 0xF0) | (2 << 2)) }),
            Cycle::Cycle63 => rb
                .pwm_config()
                .modify(|r, w| unsafe { w.bits((r.bits() & 0xF0) | (2 << 2) | 0x01) }),
            Cycle::Cycle32 => rb
                .pwm_config()
                .modify(|r, w| unsafe { w.bits((r.bits() & 0xF0) | (3 << 2)) }),
            Cycle::Cycle31 => rb
                .pwm_config()
                .modify(|r, w| unsafe { w.bits((r.bits() & 0xF0) | (3 << 2) | 0x01) }),
        }
    }
}

impl<'d, T: Instance> Drop for Pwm<'d, T> {
    fn drop(&mut self) {
        let rb = T::regs();
        rb.pwm_out_en().write(|w| unsafe { w.bits(0) });
    }
}

pub(crate) mod sealed {

    pub trait Instance {
        type Interrupt: crate::interrupt::Interrupt;

        /// Return the [`RegisterBlock`] of the pwmx peripheral
        fn regs() -> &'static crate::pac::pwmx::RegisterBlock;
    }

    pub trait PwmPin<T: Instance> {
        /// Return the channel of the specified [`PwmPin`]
        fn channel(&self) -> crate::pwm::Channel;
    }
}

pub trait Instance: sealed::Instance + crate::Peripheral<P = Self> {}
pub trait PwmPin<T: Instance>: sealed::PwmPin<T> {}

impl sealed::Instance for PWMX {
    type Interrupt = crate::interrupt::PWMX;

    fn regs() -> &'static crate::pac::pwmx::RegisterBlock {
        unsafe { &*pac::PWMX::PTR }
    }
}
impl Instance for PWMX {}

macro_rules! impl_pwm_pin {
    ($inst:ident, $pin:ident, $ch:expr) => {
        impl crate::pwm::PwmPin<$inst> for crate::gpio::Output<'_, $pin> {}
        impl crate::pwm::sealed::PwmPin<$inst> for crate::gpio::Output<'_, $pin> {
            /// Return the channel of the specified [`PwmPin`]
            fn channel(&self) -> crate::pwm::Channel {
                $ch
            }
        }
    };
}

impl_pwm_pin!(PWMX, PA6, Channel::Pwm4);
impl_pwm_pin!(PWMX, PA12, Channel::Pwm4);
impl_pwm_pin!(PWMX, PA13, Channel::Pwm5);
impl_pwm_pin!(PWMX, PA7, Channel::Pwm5);
impl_pwm_pin!(PWMX, PB0, Channel::Pwm6);
impl_pwm_pin!(PWMX, PB1, Channel::Pwm7);
impl_pwm_pin!(PWMX, PB4, Channel::Pwm7);
impl_pwm_pin!(PWMX, PB6, Channel::Pwm8);
impl_pwm_pin!(PWMX, PB2, Channel::Pwm8);
impl_pwm_pin!(PWMX, PB3, Channel::Pwm9);
impl_pwm_pin!(PWMX, PB7, Channel::Pwm9);
impl_pwm_pin!(PWMX, PB14, Channel::Pwm10);
impl_pwm_pin!(PWMX, PB23, Channel::Pwm11);
