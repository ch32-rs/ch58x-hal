# ch58x-hal

HAL for the CH58x RISC-V BLE microcotrollers from WCH.

This crate is under random and active development. DO NOT USE in production.

This should be the reference hal implementation for CH57x, CH58x, CH59x.

## Features

- Basic: clock init, delay, interrupt, etc.
- Dedicated runtime
- Uart Tx only
- GPIO
- RTC, with datetime support
- embassy time driver via SysTick
- SysTick
- I2C
- ADC
- [ ] SPI, not working, help-wanted

## Usage

Refer `Cargo.toml` and `examples` directory.

## References

- [Slappy2022/ch58x-hal](https://github.com/Slappy2022/ch58x-hal)
