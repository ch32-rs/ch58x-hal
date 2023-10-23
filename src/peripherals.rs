use crate::pac;

// We need to export this in the hal for the drivers to use

crate::peripherals! {
    SYSTICK <= SYSTICK,
    UART0 <= UART0,
    UART1 <= UART1,
    UART2 <= UART2,
    UART3 <= UART3,

    SPI0 <= SPI0,
    // SPI1 is only avalible to CH583

    I2C <= I2C,
    RTC <= RTC,
    GPIO <= virtual,

    ADC <= ADC,
    TKEY <= virtual,

    USB <= USB,

    TMR0 <= TMR0,
    TMR1 <= TMR1,
    TMR2 <= TMR2,
    TMR3 <= TMR3,
    PWMX <= PWMX,

    BLE <= virtual,

    PA7 <= virtual,
    PA8 <= virtual,
    PA9 <= virtual,

    PB9 <= virtual,
    PB8 <= virtual,

    PB17 <= virtual,
    PB16 <= virtual,
    PB15 <= virtual,
    PB14 <= virtual,
    PB13 <= virtual,
    PB12 <= virtual,
    PB11 <= virtual,
    PB10 <= virtual,

    PB7 <= virtual,
    PB6 <= virtual,
    PB5 <= virtual,
    PB4 <= virtual,
    PB3 <= virtual,
    PB2 <= virtual,
    PB1 <= virtual,
    PB0 <= virtual,

    PB23 <= virtual,
    PB22 <= virtual,
    PB21 <= virtual,
    PB20 <= virtual,
    PB19 <= virtual,
    PB18 <= virtual,

    PA4 <= virtual,
    PA5 <= virtual,
    PA6 <= virtual,

    PA0 <= virtual,
    PA1 <= virtual,
    PA2 <= virtual,
    PA3 <= virtual,

    PA15 <= virtual,
    PA14 <= virtual,
    PA13 <= virtual,
    PA12 <= virtual,
    PA11 <= virtual,
    PA10 <= virtual,
}
