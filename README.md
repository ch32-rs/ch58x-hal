# ch58x-hal

[![Github Actions][github-workflow]][homepage]
[![Crates.io][badge-license]][crates]
[![Crates.io][badge-version]][crates]
[![docs.rs][badge-docsrs]][docsrs]

[github-workflow]: https://img.shields.io/github/actions/workflow/status/ch32-rs/ch58x-hal/rust.yml?style=for-the-badge
[badge-license]: https://img.shields.io/crates/l/ch58x-hal?style=for-the-badge
[badge-version]: https://img.shields.io/crates/v/ch58x-hal?style=for-the-badge
[badge-docsrs]: https://img.shields.io/docsrs/ch58x-hal?style=for-the-badge
[crates]: https://crates.io/crates/ch58x-hal
[docsrs]: https://docs.rs/ch58x-hal
[homepage]: https://github.com/ch32-rs/ch58x-hal

HAL for the CH58x RISC-V BLE microcotrollers from WCH.

This crate is under random and active development. DO NOT USE in production.

This should be the reference hal implementation for CH57x, CH58x, CH59x.

## Features

- Basic: clock init, delay, interrupt, etc.
- Dedicated runtime: interrupt table, hardware stack push, highcode support, critical section implementation
- embassy time driver via SysTick, defaults to 32KHz tick
- GPIO, with async support
- UART, basic blocking tx, rx
- RTC, with a nice datetime support
- SysTick (conflicts with embassy time driver)
- I2C
- ADC, Temperature sensor, VBAT sensor
- SPI
- libISP ROM functions

## Usage

Refer `Cargo.toml` and `examples` directory.

## Notes

- `UNDOCUMENTED:` tags in code comments means the information is not from official documents.

## References

- [Slappy2022/ch58x-hal](https://github.com/Slappy2022/ch58x-hal)
- [Slappy2022/ch58x-ble-rt](https://github.com/Slappy2022/ch58x-ble-rt)
