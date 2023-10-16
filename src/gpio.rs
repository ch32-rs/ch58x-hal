/*!
    GPIO_ModeIN_Floating, //浮空输入
    GPIO_ModeIN_PU,       //上拉输入
    GPIO_ModeIN_PD,       //下拉输入
    GPIO_ModeOut_PP_5mA,  //推挽输出最大5mA
    GPIO_ModeOut_PP_20mA, //推挽输出最大20mA
*/

#![macro_use]

use crate::{impl_peripheral, into_ref, pac, peripherals, Peripheral, PeripheralRef};

/// GPIO flexible pin.
///
/// This pin can either be a disconnected, input, or output pin, or both. The level register bit will remain
/// set while not in output mode, so the pin's level will be 'remembered' when it is not in output
/// mode.
pub struct Flex<'d, T: Pin> {
    pub(crate) pin: PeripheralRef<'d, T>,
}

impl<'d, T: Pin> Flex<'d, T> {
    /// Wrap the pin in a `Flex`.
    ///
    /// The pin remains disconnected. The initial output level is unspecified, but can be changed
    /// before the pin is put into output mode.
    ///
    #[inline]
    pub fn new(pin: impl Peripheral<P = T> + 'd) -> Self {
        into_ref!(pin);
        // Pin will be in disconnected state.
        Self { pin }
    }

    #[inline]
    pub fn degrade(self) -> Flex<'d, AnyPin> {
        // Safety: We are about to drop the other copy of this pin, so
        // this clone is safe.
        let pin = unsafe { self.pin.clone_unchecked() };

        // We don't want to run the destructor here, because that would
        // deconfigure the pin.
        core::mem::forget(self);

        Flex {
            pin: pin.map_into::<AnyPin>(),
        }
    }

    /// Put the pin into input mode.
    #[inline]
    pub fn set_as_input(&mut self, pull: Pull) {
        let n = self.pin.pin() as usize;
        let gpio = unsafe { &*pac::GPIO::PTR };
        critical_section::with(|_| {
            match self.pin.port() {
                0 => {
                    match pull {
                        Pull::None => unsafe {
                            // In_floating
                            gpio.pa_pd_drv.modify(|r, w| w.bits(r.bits() & !(1 << n)));
                            gpio.pa_pu.modify(|r, w| w.bits(r.bits() & !(1 << n)));
                            gpio.pa_dir.modify(|r, w| w.bits(r.bits() & !(1 << n)));
                        },
                        Pull::Up => unsafe {
                            // In_PU
                            gpio.pa_pd_drv.modify(|r, w| w.bits(r.bits() & !(1 << n)));
                            gpio.pa_pu.modify(|r, w| w.bits(r.bits() | (1 << n)));
                            gpio.pa_dir.modify(|r, w| w.bits(r.bits() & !(1 << n)));
                        },
                        Pull::Down => unsafe {
                            // In_PD
                            gpio.pa_pd_drv.modify(|r, w| w.bits(r.bits() | (1 << n)));
                            gpio.pa_pu.modify(|r, w| w.bits(r.bits() & !(1 << n)));
                            gpio.pa_dir.modify(|r, w| w.bits(r.bits() & !(1 << n)));
                        },
                    }
                }
                // PB
                1 => match pull {
                    Pull::None => unsafe {
                        gpio.pb_pd_drv.modify(|r, w| w.bits(r.bits() & !(1 << n)));
                        gpio.pb_pu.modify(|r, w| w.bits(r.bits() & !(1 << n)));
                        gpio.pb_dir.modify(|r, w| w.bits(r.bits() & !(1 << n)));
                    },
                    Pull::Up => unsafe {
                        gpio.pb_pd_drv.modify(|r, w| w.bits(r.bits() & !(1 << n)));
                        gpio.pb_pu.modify(|r, w| w.bits(r.bits() | (1 << n)));
                        gpio.pb_dir.modify(|r, w| w.bits(r.bits() & !(1 << n)));
                    },
                    Pull::Down => unsafe {
                        gpio.pb_pd_drv.modify(|r, w| w.bits(r.bits() | (1 << n)));
                        gpio.pb_pu.modify(|r, w| w.bits(r.bits() & !(1 << n)));
                        gpio.pb_dir.modify(|r, w| w.bits(r.bits() & !(1 << n)));
                    },
                },
                _ => unreachable!(),
            }
        });
    }

    /// Put the pin into output mode.
    ///
    /// The pin level will be whatever was set before (or low by default). If you want it to begin
    /// at a specific level, call `set_high`/`set_low` on the pin first.
    #[inline]
    pub fn set_as_output(&mut self, drive: OutputDrive) {
        critical_section::with(|_| {
            let gpio = unsafe { &*pac::GPIO::PTR };
            let n = self.pin.pin();
            match self.pin.port() {
                0 => match drive {
                    OutputDrive::Low => unsafe {
                        gpio.pa_pd_drv.modify(|r, w| w.bits(r.bits() & !(1 << n)));
                        gpio.pa_dir.modify(|r, w| w.bits(r.bits() | (1 << n)));
                    },
                    OutputDrive::High => unsafe {
                        gpio.pa_pd_drv.modify(|r, w| w.bits(r.bits() | (1 << n)));
                        gpio.pa_dir.modify(|r, w| w.bits(r.bits() | (1 << n)));
                    },
                },
                1 => match drive {
                    OutputDrive::Low => unsafe {
                        gpio.pb_pd_drv.modify(|r, w| w.bits(r.bits() & !(1 << n)));
                        gpio.pb_dir.modify(|r, w| w.bits(r.bits() | (1 << n)));
                    },
                    OutputDrive::High => unsafe {
                        gpio.pb_pd_drv.modify(|r, w| w.bits(r.bits() | (1 << n)));
                        gpio.pb_dir.modify(|r, w| w.bits(r.bits() | (1 << n)));
                    },
                },
                _ => unreachable!(),
            }
        });
    }

    #[inline]
    pub fn is_high(&self) -> bool {
        !self.is_low()
    }

    #[inline]
    pub fn is_low(&self) -> bool {
        let gpio = unsafe { &*pac::GPIO::PTR };
        match self.pin.port() {
            0 => gpio.pa_pin.read().bits() & (1 << self.pin.pin()) == 0,
            1 => gpio.pb_pin.read().bits() & (1 << self.pin.pin()) == 0,
            _ => unreachable!(),
        }
    }

    #[inline]
    pub fn get_level(&self) -> Level {
        self.is_high().into()
    }

    #[inline]
    pub fn is_set_high(&self) -> bool {
        !self.is_set_low()
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        let gpio = unsafe { &*pac::GPIO::PTR };
        let mask = 1 << self.pin.pin();
        match self.pin.port() {
            0 => gpio.pa_out.read().bits() & mask == 0,
            1 => gpio.pb_out.read().bits() & mask == 0,
            _ => unreachable!(),
        }
    }

    /// What level output is set to
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.is_set_high().into()
    }

    #[inline]
    pub fn set_high(&mut self) {
        self.pin.set_high();
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        self.pin.set_low();
    }

    #[inline]
    pub fn set_level(&mut self, level: Level) {
        match level {
            Level::Low => self.pin.set_low(),
            Level::High => self.pin.set_high(),
        }
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        if self.is_set_low() {
            self.set_high()
        } else {
            self.set_low()
        }
    }
}

// TOOD: Drop

/// Pull setting for an input.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Pull {
    None,
    Up,
    Down,
}

/// Drive current settings for PushPull outputs.
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OutputDrive {
    // The drive current is 5mA
    Low,
    // The drive current is 20mA
    High,
}

/// GPIO input driver.
pub struct Input<'d, T: Pin> {
    pub(crate) pin: Flex<'d, T>,
}

impl<'d, T: Pin> Input<'d, T> {
    #[inline]
    pub fn new(pin: impl Peripheral<P = T> + 'd, pull: Pull) -> Self {
        let mut pin = Flex::new(pin);
        pin.set_as_input(pull);
        Self { pin }
    }

    #[inline]
    pub fn degrade(self) -> Input<'d, AnyPin> {
        Input {
            pin: self.pin.degrade(),
        }
    }

    #[inline]
    pub fn is_high(&self) -> bool {
        self.pin.is_high()
    }

    #[inline]
    pub fn is_low(&self) -> bool {
        self.pin.is_low()
    }

    #[inline]
    pub fn get_level(&self) -> Level {
        self.pin.get_level()
    }
}

/// Digital input or output level.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Level {
    Low,
    High,
}

impl From<bool> for Level {
    fn from(val: bool) -> Self {
        match val {
            true => Self::High,
            false => Self::Low,
        }
    }
}

impl From<Level> for bool {
    fn from(level: Level) -> bool {
        match level {
            Level::Low => false,
            Level::High => true,
        }
    }
}

/// GPIO output driver.
///
/// Note that pins will **return to their floating state** when `Output` is dropped.
/// If pins should retain their state indefinitely, either keep ownership of the
/// `Output`, or pass it to [`core::mem::forget`].
pub struct Output<'d, T: Pin> {
    pub(crate) pin: Flex<'d, T>,
}

impl<'d, T: Pin> Output<'d, T> {
    #[inline]
    pub fn new(pin: impl Peripheral<P = T> + 'd, initial_output: Level, drive: OutputDrive) -> Self {
        let mut pin = Flex::new(pin);
        match initial_output {
            Level::High => pin.set_high(),
            Level::Low => pin.set_low(),
        }
        pin.set_as_output(drive);
        Self { pin }
    }

    #[inline]
    pub fn degrade(self) -> Output<'d, AnyPin> {
        Output {
            pin: self.pin.degrade(),
        }
    }

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        self.pin.set_high();
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        self.pin.set_low();
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_level(level)
    }

    /// Is the output pin set as high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.pin.is_set_high()
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.pin.is_set_low()
    }

    /// What level output is set to
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.pin.get_output_level()
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        self.pin.toggle();
    }
}

// No OpenDrain for CH59x

pub(crate) mod sealed {
    use super::*;

    pub trait Pin {
        fn pin_port(&self) -> u8;

        #[inline]
        fn _pin(&self) -> u8 {
            self.pin_port() % 32
        }
        #[inline]
        fn _port(&self) -> u8 {
            self.pin_port() / 32
        }

        /// Set the output as high.
        #[inline]
        fn set_high(&self) {
            let gpio = unsafe { &*pac::GPIO::PTR };
            let n = self._pin();
            match self._port() {
                0 => unsafe {
                    gpio.pa_out.modify(|r, w| w.bits(r.bits() | (1 << n)));
                },
                1 => unsafe {
                    gpio.pb_out.modify(|r, w| w.bits(r.bits() | (1 << n)));
                },
                _ => unreachable!(),
            }
        }

        /// Set the output as low.
        #[inline]
        fn set_low(&self) {
            let gpio = unsafe { &*pac::GPIO::PTR };
            let n = self._pin();
            match self._port() {
                0 => unsafe {
                    gpio.pa_clr.modify(|r, w| w.bits(r.bits() | (1 << n)));
                },
                1 => unsafe {
                    gpio.pb_clr.modify(|r, w| w.bits(r.bits() | (1 << n)));
                },
                _ => unreachable!(),
            }
        }

        #[inline]
        fn set_as_analog(&self) {
            // GPIO_ModeIN_Floating
            let gpio = unsafe { &*pac::GPIO::PTR };
            let pin = self._pin() as usize;
            match self._port() {
                0 => unsafe {
                    gpio.pa_pd_drv.modify(|r, w| w.bits(r.bits() & !(1 << pin)));
                    gpio.pa_pu.modify(|r, w| w.bits(r.bits() & !(1 << pin)));
                    gpio.pa_dir.modify(|r, w| w.bits(r.bits() & !(1 << pin)));
                },
                1 => unsafe {
                    gpio.pb_pd_drv.modify(|r, w| w.bits(r.bits() & !(1 << pin)));
                    gpio.pb_pu.modify(|r, w| w.bits(r.bits() & !(1 << pin)));
                    gpio.pb_dir.modify(|r, w| w.bits(r.bits() & !(1 << pin)));
                },
                _ => unreachable!(),
            }
        }

        /// Set the pin as an input, for peripherals functions
        #[inline]
        fn set_as_output_with_drive_low(&self) {
            let gpio = unsafe { &*pac::GPIO::PTR };
            let pin = self._pin() as usize;
            match self._port() {
                0 => unsafe {
                    gpio.pa_dir.modify(|r, w| w.bits(r.bits() | (1 << pin)));
                    gpio.pa_pd_drv.modify(|r, w| w.bits(r.bits() & !(1 << pin)));
                },
                1 => unsafe {
                    gpio.pb_dir.modify(|r, w| w.bits(r.bits() & !(1 << pin)));
                    gpio.pb_pd_drv.modify(|r, w| w.bits(r.bits() & !(1 << pin)));
                },
                _ => unreachable!(),
            }
        }

        #[inline]
        fn set_drive(&self, drive: OutputDrive) {
            let gpio = unsafe { &*pac::GPIO::PTR };
            let pin = self._pin() as usize;
            match self._port() {
                0 => match drive {
                    OutputDrive::Low => unsafe {
                        gpio.pa_pd_drv.modify(|r, w| w.bits(r.bits() & !(1 << pin)));
                    },
                    OutputDrive::High => unsafe {
                        gpio.pa_pd_drv.modify(|r, w| w.bits(r.bits() | (1 << pin)));
                    },
                },
                1 => match drive {
                    OutputDrive::Low => unsafe {
                        gpio.pb_pd_drv.modify(|r, w| w.bits(r.bits() & !(1 << pin)));
                    },
                    OutputDrive::High => unsafe {
                        gpio.pb_pd_drv.modify(|r, w| w.bits(r.bits() | (1 << pin)));
                    },
                },
                _ => unreachable!(),
            }
        }
    }
}

pub trait Pin: Peripheral<P = Self> + Into<AnyPin> + sealed::Pin + Sized + 'static {
    /// Number of the pin within the port (0..31)
    #[inline]
    fn pin(&self) -> u8 {
        self._pin()
    }

    /// Port of the pin
    #[inline]
    fn port(&self) -> u8 {
        self._port()
    }

    /// Convert from concrete pin type PX_XX to type erased `AnyPin`.
    #[inline]
    fn degrade(self) -> AnyPin {
        AnyPin {
            pin_port: self.pin_port(),
        }
    }
}

// Type-erased GPIO pin
pub struct AnyPin {
    pin_port: u8,
}

impl AnyPin {
    #[inline]
    pub unsafe fn steal(pin_port: u8) -> Self {
        Self { pin_port }
    }

    #[inline]
    fn _port(&self) -> u8 {
        self.pin_port / 32
    }
}

impl_peripheral!(AnyPin);
impl Pin for AnyPin {}
impl sealed::Pin for AnyPin {
    #[inline]
    fn pin_port(&self) -> u8 {
        self.pin_port
    }
}

macro_rules! foreach_pin {
    ($($pat:tt => $code:tt;)*) => {
        macro_rules! __foreach_pin_inner {
            $(($pat) => $code;)*
            ($_:tt) => {}
        }
        __foreach_pin_inner!((PA4,GPIOA,0,4));
        __foreach_pin_inner!((PA5,GPIOA,0,5));
        __foreach_pin_inner!((PA6,GPIOA,0,6));
        __foreach_pin_inner!((PA7,GPIOA,0,7));
        __foreach_pin_inner!((PA8,GPIOA,0,8));
        __foreach_pin_inner!((PA9,GPIOA,0,9));
        __foreach_pin_inner!((PA10,GPIOA,0,10));
        __foreach_pin_inner!((PA11,GPIOA,0,11));
        __foreach_pin_inner!((PA12,GPIOA,0,12));
        __foreach_pin_inner!((PA13,GPIOA,0,13));
        __foreach_pin_inner!((PA14,GPIOA,0,14));
        __foreach_pin_inner!((PA15,GPIOA,0,15));
        __foreach_pin_inner!((PB0,GPIOB,1,0));
        __foreach_pin_inner!((PB4,GPIOB,1,4));
        __foreach_pin_inner!((PB6,GPIOB,1,6));
        __foreach_pin_inner!((PB7,GPIOB,1,7));
        __foreach_pin_inner!((PB10,GPIOB,1,10));
        __foreach_pin_inner!((PB11,GPIOB,1,11));
        __foreach_pin_inner!((PB12,GPIOB,1,12));
        __foreach_pin_inner!((PB13,GPIOB,1,13));
        __foreach_pin_inner!((PB14,GPIOB,1,14));
        __foreach_pin_inner!((PB15,GPIOB,1,15));
        __foreach_pin_inner!((PB20,GPIOB,1,20));
        __foreach_pin_inner!((PB21,GPIOB,1,21));
        __foreach_pin_inner!((PB22,GPIOB,1,22));
        __foreach_pin_inner!((PB23,GPIOB,1,23));
    };
}
foreach_pin!(
    ($pin_name:ident, $port_name:ident, $port_num:expr, $pin_num:expr) => {
        impl Pin for peripherals::$pin_name {}

        impl sealed::Pin for peripherals::$pin_name {
            #[inline]
            fn pin_port(&self) -> u8 {
                $port_num * 32 + $pin_num
            }
        }

        impl From<peripherals::$pin_name> for AnyPin {
            fn from(x: peripherals::$pin_name) -> Self {
                x.degrade()
           }
        }
    };
);
