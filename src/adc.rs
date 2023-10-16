use core::ptr;

/// Convert ADC data to temperature in milli celsius.
pub fn adc_to_temperature_milli_celsius(data: u16) -> i32 {
    const ROM_CFG_TMP_25C: *const u32 = 0x7F014 as *const u32;

    let c25 = unsafe { ptr::read_volatile(ROM_CFG_TMP_25C) };

    // current temperature = standard temperature + (adc deviation * adc linearity coefficient)
    //  temp = (((C25 >> 16) & 0xFFFF) ? ((C25 >> 16) & 0xFFFF) : 25) + \
    // (adc_val - ((int)(C25 & 0xFFFF))) * 10 / 27;
    let c25_ = ((c25 >> 16) & 0xFFFF) as i32;
    let c25_ = if c25 != 0 { c25_ } else { 25 };
    c25_ * 1000 + ((data as i32) - ((c25 & 0xFFFF) as i32)) * 10_000 / 27
}
