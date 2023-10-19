use crate::pac;

// We need to export this in the hal for the drivers to use

crate::peripherals! {
    SYSTICK <= SYSTICK,
    UART0 <= UART0,
    UART1 <= UART1,
    UART2 <= UART2,
    UART3 <= UART3,
    SPI0 <= SPI0,
    I2C <= I2C,
    RTC <= RTC,
    GPIO <= virtual,
    ADC <= ADC,
    USB <= USB,
    TMR0 <= TMR0,
    TMR1 <= TMR1,
    TMR2 <= TMR2,
    TMR3 <= TMR3,
    TKEY <= virtual,
    BLE <= virtual,
    PWMX <= PWMX,

    ADC_TEMP_SENSOR <= virtual,
    ADC_VBAT_SENSOR <= virtual,

    PA4 <= virtual,
    PA5 <= virtual,
    PA6 <= virtual,
    PA7 <= virtual,
    PA8 <= virtual,
    PA9 <= virtual,
    PA10 <= virtual,
    PA11 <= virtual,
    PA12 <= virtual,
    PA13 <= virtual,
    PA14 <= virtual,
    PA15 <= virtual,

    PB0 <= virtual,
    PB4 <= virtual,
    PB6 <= virtual,
    PB7 <= virtual,
    PB10 <= virtual,
    PB11 <= virtual,
    PB12 <= virtual,
    PB13 <= virtual,
    PB14 <= virtual,
    PB15 <= virtual,
    PB20 <= virtual,
    PB21 <= virtual, // noex in CH59x
    PB22 <= virtual,
    PB23 <= virtual,

    // CH582
    PA0 <= virtual,
    PA1 <= virtual,
    PA2 <= virtual,
    PA3 <= virtual,
}
