#![no_std]
#![no_main]

use embedded_hal_1::delay::DelayUs;
use hal::gpio::{Input, Level, Output, OutputDrive, Pull};
use hal::i2c::{self, I2c};
use hal::prelude::*;
// use hal::interrupt::Interrupt;
use hal::rtc::{DateTime, Rtc};
use hal::systick::SysTick;
use hal::uart::UartTx;
use hal::{delay_ms, pac, peripherals, with_safe_access};
use {ch58x_hal as hal, panic_halt as _};

static mut SERIAL: Option<UartTx<peripherals::UART1>> = None;

macro_rules! println {
    ($($arg:tt)*) => {
        unsafe {
            use core::fmt::Write;
            use core::writeln;

            if let Some(uart) = SERIAL.as_mut() {
                writeln!(uart, $($arg)*).unwrap();
            }
        }
    }
}

const MPU6050_ADDR: u8 = 0x68;

pub struct MPU6050<'d> {
    i2c: I2c<'d, peripherals::I2C>,
    addr: u8,
}

#[derive(Debug)]
pub enum MPU6050Error {
    I2CError(i2c::Error),
    WrongDevice,
}

impl From<i2c::Error> for MPU6050Error {
    fn from(e: i2c::Error) -> Self {
        Self::I2CError(e)
    }
}

#[allow(unused)]
mod regs {
    pub const XGOFFS_TC: u8 = 0x00; // Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD
    pub const YGOFFS_TC: u8 = 0x01;
    pub const ZGOFFS_TC: u8 = 0x02;
    pub const X_FINE_GAIN: u8 = 0x03; // [7:0] fine gain
    pub const Y_FINE_GAIN: u8 = 0x04;
    pub const Z_FINE_GAIN: u8 = 0x05;
    pub const XA_OFFSET_H: u8 = 0x06; // User-defined trim values for accelerometer
    pub const XA_OFFSET_L_TC: u8 = 0x07;
    pub const YA_OFFSET_H: u8 = 0x08;
    pub const YA_OFFSET_L_TC: u8 = 0x09;
    pub const ZA_OFFSET_H: u8 = 0x0A;
    pub const ZA_OFFSET_L_TC: u8 = 0x0B;
    pub const SELF_TEST_X: u8 = 0x0D;
    pub const SELF_TEST_Y: u8 = 0x0E;
    pub const SELF_TEST_Z: u8 = 0x0F;
    pub const SELF_TEST_A: u8 = 0x10;
    pub const XG_OFFS_USRH: u8 = 0x13; // User-defined trim values for gyroscope; supported in MPU-6050?
    pub const XG_OFFS_USRL: u8 = 0x14;
    pub const YG_OFFS_USRH: u8 = 0x15;
    pub const YG_OFFS_USRL: u8 = 0x16;
    pub const ZG_OFFS_USRH: u8 = 0x17;
    pub const ZG_OFFS_USRL: u8 = 0x18;
    pub const SMPLRT_DIV: u8 = 0x19;
    pub const CONFIG: u8 = 0x1A;
    pub const GYRO_CONFIG: u8 = 0x1B;
    pub const ACCEL_CONFIG: u8 = 0x1C;
    pub const FF_THR: u8 = 0x1D; // Free-fall
    pub const FF_DUR: u8 = 0x1E; // Free-fall
    pub const MOT_THR: u8 = 0x1F; // Motion detection threshold bits [7:0]
    pub const MOT_DUR: u8 = 0x20; // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
    pub const ZMOT_THR: u8 = 0x21; // Zero-motion detection threshold bits [7:0]
    pub const ZRMOT_DUR: u8 = 0x22; // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
    pub const FIFO_EN: u8 = 0x23;
    pub const I2C_MST_CTRL: u8 = 0x24;
    pub const I2C_SLV0_ADDR: u8 = 0x25;
    pub const I2C_SLV0_REG: u8 = 0x26;
    pub const I2C_SLV0_CTRL: u8 = 0x27;
    pub const I2C_SLV1_ADDR: u8 = 0x28;
    pub const I2C_SLV1_REG: u8 = 0x29;
    pub const I2C_SLV1_CTRL: u8 = 0x2A;
    pub const I2C_SLV2_ADDR: u8 = 0x2B;
    pub const I2C_SLV2_REG: u8 = 0x2C;
    pub const I2C_SLV2_CTRL: u8 = 0x2D;
    pub const I2C_SLV3_ADDR: u8 = 0x2E;
    pub const I2C_SLV3_REG: u8 = 0x2F;
    pub const I2C_SLV3_CTRL: u8 = 0x30;
    pub const I2C_SLV4_ADDR: u8 = 0x31;
    pub const I2C_SLV4_REG: u8 = 0x32;
    pub const I2C_SLV4_DO: u8 = 0x33;
    pub const I2C_SLV4_CTRL: u8 = 0x34;
    pub const I2C_SLV4_DI: u8 = 0x35;
    pub const I2C_MST_STATUS: u8 = 0x36;
    pub const INT_PIN_CFG: u8 = 0x37;
    pub const INT_ENABLE: u8 = 0x38;
    pub const DMP_INT_STATUS: u8 = 0x39; // Check DMP interrupt
    pub const INT_STATUS: u8 = 0x3A;
    pub const ACCEL_XOUT_H: u8 = 0x3B;
    pub const ACCEL_XOUT_L: u8 = 0x3C;
    pub const ACCEL_YOUT_H: u8 = 0x3D;
    pub const ACCEL_YOUT_L: u8 = 0x3E;
    pub const ACCEL_ZOUT_H: u8 = 0x3F;
    pub const ACCEL_ZOUT_L: u8 = 0x40;
    pub const TEMP_OUT_H: u8 = 0x41;
    pub const TEMP_OUT_L: u8 = 0x42;
    pub const GYRO_XOUT_H: u8 = 0x43;
    pub const GYRO_XOUT_L: u8 = 0x44;
    pub const GYRO_YOUT_H: u8 = 0x45;
    pub const GYRO_YOUT_L: u8 = 0x46;
    pub const GYRO_ZOUT_H: u8 = 0x47;
    pub const GYRO_ZOUT_L: u8 = 0x48;
    pub const EXT_SENS_DATA_00: u8 = 0x49;
    pub const EXT_SENS_DATA_01: u8 = 0x4A;
    pub const EXT_SENS_DATA_02: u8 = 0x4B;
    pub const EXT_SENS_DATA_03: u8 = 0x4C;
    pub const EXT_SENS_DATA_04: u8 = 0x4D;
    pub const EXT_SENS_DATA_05: u8 = 0x4E;
    pub const EXT_SENS_DATA_06: u8 = 0x4F;
    pub const EXT_SENS_DATA_07: u8 = 0x50;
    pub const EXT_SENS_DATA_08: u8 = 0x51;
    pub const EXT_SENS_DATA_09: u8 = 0x52;
    pub const EXT_SENS_DATA_10: u8 = 0x53;
    pub const EXT_SENS_DATA_11: u8 = 0x54;
    pub const EXT_SENS_DATA_12: u8 = 0x55;
    pub const EXT_SENS_DATA_13: u8 = 0x56;
    pub const EXT_SENS_DATA_14: u8 = 0x57;
    pub const EXT_SENS_DATA_15: u8 = 0x58;
    pub const EXT_SENS_DATA_16: u8 = 0x59;
    pub const EXT_SENS_DATA_17: u8 = 0x5A;
    pub const EXT_SENS_DATA_18: u8 = 0x5B;
    pub const EXT_SENS_DATA_19: u8 = 0x5C;
    pub const EXT_SENS_DATA_20: u8 = 0x5D;
    pub const EXT_SENS_DATA_21: u8 = 0x5E;
    pub const EXT_SENS_DATA_22: u8 = 0x5F;
    pub const EXT_SENS_DATA_23: u8 = 0x60;
    pub const MOT_DETECT_STATUS: u8 = 0x61;
    pub const I2C_SLV0_DO: u8 = 0x63;
    pub const I2C_SLV1_DO: u8 = 0x64;
    pub const I2C_SLV2_DO: u8 = 0x65;
    pub const I2C_SLV3_DO: u8 = 0x66;
    pub const I2C_MST_DELAY_CTRL: u8 = 0x67;
    pub const SIGNAL_PATH_RESET: u8 = 0x68;
    pub const MOT_DETECT_CTRL: u8 = 0x69;
    pub const USER_CTRL: u8 = 0x6A; // Bit 7 enable DMP, bit 3 reset DMP
    pub const PWR_MGMT_1: u8 = 0x6B; // Device defaults to the SLEEP mode
    pub const PWR_MGMT_2: u8 = 0x6C;
    pub const DMP_BANK: u8 = 0x6D; // Activates a specific bank in the DMP
    pub const DMP_RW_PNT: u8 = 0x6E; // Set read/write pointer to a specific start address in specified DMP bank
    pub const DMP_REG: u8 = 0x6F; // Register in DMP from which to read or to which to write
    pub const DMP_REG_1: u8 = 0x70;
    pub const DMP_REG_2: u8 = 0x71;
    pub const FIFO_COUNTH: u8 = 0x72;
    pub const FIFO_COUNTL: u8 = 0x73;
    pub const FIFO_R_W: u8 = 0x74;
    pub const WHO_AM_I_MPU6050: u8 = 0x75; // Should return 0x68
}

#[derive(Debug, Clone, Copy)]
pub enum AccelScale {
    AFS2G = 0,
    AFS4G,
    AFS8G,
    AFS16G,
}

#[derive(Debug, Clone, Copy)]
pub enum GyroScale {
    GFS250DPS = 0,
    GFS500DPS,
    GFS1000DPS,
    GFS2000DPS,
}

pub struct Config {
    pub gyro_scale: GyroScale,
    pub accel_scale: AccelScale,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            gyro_scale: GyroScale::GFS250DPS,
            accel_scale: AccelScale::AFS2G,
        }
    }
}

// int Gscale = GFS_250DPS;
// int Ascale = AFS_2G;

// REF: https://github.com/kriswiner/MPU6050/blob/master/MPU6050Library/MPU6050.cpp
impl<'d> MPU6050<'d> {
    pub fn new(i2c: I2c<'d, peripherals::I2C>, config: Config) -> Result<Self, MPU6050Error> {
        let mut this = Self {
            i2c,
            addr: MPU6050_ADDR,
        };

        this.init(&config)?;

        Ok(this)
    }

    pub fn read_accel(&mut self) -> Result<[i16; 3], MPU6050Error> {
        let mut buf = [0u8; 6];
        self.i2c
            .blocking_write_read(self.addr, &[regs::ACCEL_XOUT_H], &mut buf)?;
        let accel_raw = [
            ((buf[0] as u16) << 8) | buf[1] as u16,
            ((buf[2] as u16) << 8) | buf[3] as u16,
            ((buf[4] as u16) << 8) | buf[5] as u16,
        ];
        let accel = [
            (accel_raw[0] as i16) >> 4,
            (accel_raw[1] as i16) >> 4,
            (accel_raw[2] as i16) >> 4,
        ];
        Ok(accel)
    }

    pub fn read_gyro(&mut self) -> Result<[i16; 3], MPU6050Error> {
        let mut buf = [0u8; 6];
        self.i2c
            .blocking_write_read(self.addr, &[regs::GYRO_XOUT_H], &mut buf)?;
        let gyro_raw = [
            ((buf[0] as u16) << 8) | buf[1] as u16,
            ((buf[2] as u16) << 8) | buf[3] as u16,
            ((buf[4] as u16) << 8) | buf[5] as u16,
        ];
        let gyro = [
            (gyro_raw[0] as i16) >> 4,
            (gyro_raw[1] as i16) >> 4,
            (gyro_raw[2] as i16) >> 4,
        ];
        Ok(gyro)
    }

    pub fn read_temperture(&mut self) -> Result<f32, MPU6050Error> {
        let mut buf = [0u8; 2];
        self.i2c.blocking_write_read(self.addr, &[regs::TEMP_OUT_H], &mut buf)?;
        let temp_raw = ((buf[0] as u16) << 8) | buf[1] as u16;
        let temp = (temp_raw as f32) / 340.0 + 36.53;
        Ok(temp)
    }

    /// Calibrate gyro and accelerometers, load biases in bias registers
    fn calibrate(&mut self, gyro_bias: [f32; 3], accel_bias: [f32; 3]) -> Result<(), MPU6050Error> {
        Ok(())
    }

    /// Initialize device for active mode read of acclerometer, gyroscope, and temperature
    fn init(&mut self, config: &Config) -> Result<(), MPU6050Error> {
        if self.read_byte(regs::WHO_AM_I_MPU6050)? != 0x68 {
            return Err(MPU6050Error::WrongDevice);
        }

        // get stable time source
        self.write_byte(regs::PWR_MGMT_1, 0x01)?; // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

        // Configure Gyro and Accelerometer
        // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
        // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
        // Maximum delay time is 4.9 ms corresponding to just over 200 Hz sample rate
        self.write_byte(regs::CONFIG, 0x03)?;

        // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
        self.write_byte(regs::SMPLRT_DIV, 0x04)?; // Use a 200 Hz rate; the same rate set in CONFIG above

        // Set gyroscope full scale range
        // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
        let c = self.read_byte(regs::GYRO_CONFIG)?;
        self.write_byte(regs::GYRO_CONFIG, c & !0xE0)?; // Clear self-test bits [7:5]
        self.write_byte(regs::GYRO_CONFIG, c & !0x18)?; // Clear AFS bits [4:3]
        self.write_byte(regs::GYRO_CONFIG, c | (config.gyro_scale as u8) << 3)?; // Set full scale range for the gyro

        // Set accelerometer configuration
        let c = self.read_byte(regs::ACCEL_CONFIG)?;
        self.write_byte(regs::ACCEL_CONFIG, c & !0xE0)?; // Clear self-test bits [7:5]
        self.write_byte(regs::ACCEL_CONFIG, c & !0x18)?; // Clear AFS bits [4:3]
        self.write_byte(regs::ACCEL_CONFIG, c | (config.accel_scale as u8) << 3)?; // Set full scale range for the accelerometer

        // Configure Interrupts and Bypass Enable
        // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
        // can join the I2C bus and all can be controlled by the Arduino as master
        self.write_byte(regs::INT_PIN_CFG, 0x22)?;
        self.write_byte(regs::INT_ENABLE, 0x01)?; // Enable data ready (bit 0) interrupt

        Ok(())
    }

    fn read_byte(&mut self, reg_addr: u8) -> Result<u8, MPU6050Error> {
        let mut buf = [0u8; 1];
        self.i2c.blocking_write_read(self.addr, &[reg_addr], &mut buf)?;
        Ok(buf[0])
    }
    fn write_byte(&mut self, reg_addr: u8, byte: u8) -> Result<(), MPU6050Error> {
        self.i2c.blocking_write(self.addr, &[reg_addr, byte])?;
        Ok(())
    }
}

#[ch32v_rt::entry]
fn main() -> ! {
    // LED PA8

    let mut config = hal::Config::default();
    config.clock.use_pll_60mhz();
    let p = hal::init(config);

    let mut delay = SysTick::new(p.SYSTICK);

    // GPIO
    let mut led = Output::new(p.PA8, Level::Low, OutputDrive::Low);
    let download_button = Input::new(p.PB22, Pull::Up);
    let reset_button = Input::new(p.PB23, Pull::Up);

    let uart = UartTx::new(p.UART1, p.PA9, Default::default()).unwrap();
    unsafe {
        SERIAL.replace(uart);
    }

    let rtc = Rtc::new(p.RTC);

    println!("\nHello World!");
    println!("System Clocks: {}", hal::sysctl::clocks().hclk);
    println!("ChipID: 0x{:02x}", hal::signature::get_chip_id());
    println!("RTC datetime: {}", rtc.now());

    let mut i2c_config = i2c::Config::default();

    i2c_config.freq = 400.kHz();
    let i2c = I2c::new(p.I2C, p.PB13, p.PB12, i2c_config);

    let mut mpu6050 = MPU6050::new(i2c, Config::default()).unwrap();

    let temp = mpu6050.read_temperture().unwrap();
    println!("temp: {}", temp);

    loop {
        led.toggle();
        println!("tick");

        let accel = mpu6050.read_accel().unwrap();
        println!("accel: {:?}", accel);
        let gyro = mpu6050.read_gyro().unwrap();
        println!("gyro: {:?}", gyro);

        delay.delay_ms(1000);
    }
}
