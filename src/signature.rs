use core::ptr;

const R8_CHIP_ID: u32 = 0x40001041;

/// 0x92 for CH592
/// 0x91 for CH591
/// 0x81 for CH581
/// 0x82 for CH582
/// 0x83 for CH583
pub fn get_chip_id() -> u8 {
    unsafe { ptr::read_volatile(R8_CHIP_ID as *const u8) }
}
