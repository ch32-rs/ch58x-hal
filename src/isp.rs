//! The libISP583 library.
/*!
for the target in USER code area on the chip divided into USER code area and BOOT area
用于具有用户代码区和引导区的芯片、操作目标为用户代码区的情况，
可以在用户代码中被调用（IAP，擦写自身），也可以在引导代码中被调用（更新用户代码

- Flash-ROM feature:
    for store program code, support block erasing, dword and page writing, dword verifying, unit for Length is byte,
    minimal quantity for write or verify is one dword (4-bytes),
    256 bytes/page for writing, FLASH_ROM_WRITE support one dword or more dword writing, but multiple of 256 is the best,
    4KB (4096 bytes) bytes/block for erasing, so multiple of 4096 is the best

- Data-Flash(EEPROM) feature:
    for store data, support block erasing, byte and page writing, byte reading,
    minimal quantity for write or read is one byte,
    256 bytes/page for writing, EEPROM_WRITE support one byte or more byte writing, but multiple of 256 is the best,
    0.25KB/4KB (256/4096 bytes) bytes/block for erasing, so multiple of 256 or 4096 is the best */

use core::ptr;

/// Flash-ROM & Data-Flash page size for writing
pub const EEPROM_PAGE_SIZE: u32 = 256;
/// Flash-ROM & Data-Flash block size for erasing
pub const EEPROM_BLOCK_SIZE: u32 = 4096;

// DataFlash
pub const EEPROM_MIN_ERASE_SIZE: u32 = 256;
pub const EEPROM_MIN_WRITE_SIZE: u32 = 1;
pub const EEPROM_MAX_SIZE: u32 = 0x8000; // 32KB

// CodeFlash
pub const FLASH_ROM_MIN_WRITE_SIZE: u32 = 4;
pub const FLASH_ROM_MAX_SIZE: u32 = 0x070000; // 448KB

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
#[repr(u8)]
pub enum RomCmd {
    FlashRomStartIo = 0x00, // start FlashROM I/O, without parameter
    FlashRomSwReset = 0x04, // software reset FlashROM, without parameter
    GetRomInfo = 0x06,      // get information from FlashROM, parameter @Address,Buffer
    GetUniqueId = 0x07,     // get 64 bit unique ID, parameter @Buffer
    FlashRomPwrDown = 0x0D, // power-down FlashROM, without parameter
    FlashRomPwrUp = 0x0C,   // power-up FlashROM, without parameter
    FlashRomLock = 0x08,    // lock(protect)/unlock FlashROM data block, return 0 if success, parameter @StartAddr
    // StartAddr: 0=unlock all, 1=lock boot code, 3=lock all code and data
    EepromErase = 0x09,    // erase Data-Flash block, return 0 if success, parameter @StartAddr,Length
    EepromWrite = 0x0A,    // write Data-Flash data block, return 0 if success, parameter @StartAddr,Buffer,Length
    EepromRead = 0x0B,     // read Data-Flash data block, parameter @StartAddr,Buffer,Length
    FlashRomErase = 0x01,  // erase FlashROM block, return 0 if success, parameter @StartAddr,Length
    FlashRomWrite = 0x02, // write FlashROM data block, minimal block is dword, return 0 if success, parameter @StartAddr,Buffer,Length
    FlashRomVerify = 0x03, // read FlashROM data block, minimal block is dword, return 0 if success, parameter @StartAddr,Buffer,Length
}

/// address for MAC address information
const ROM_CFG_MAC_ADDR: u32 = 0x7F018;
/// address for BOOT information
const ROM_CFG_BOOT_INFO: u32 = 0x7DFF8;

#[link(name = "ISP583")]
extern "C" {
    /**
     * @brief   execute Flash/EEPROM command, caller from FlashROM or RAM
     *
     * @param   cmd         - CMD_* for caller from FlashROM or RAM.
     * @param   StartAddr   - Address of the data to be process.
     * @param   Buffer      - Pointer to the buffer where data should be process, Must be aligned to 4 bytes.
     * @param   Length      - Size of data to be process, in bytes.
     *
     * @return  0-SUCCESS  (!0)-FAILURE
     */
    pub fn FLASH_EEPROM_CMD(cmd: u8, star_addr: u32, buffer: *const u8, len: u32) -> u32;
}

/// get 64 bit unique ID
// FIXME: reads all zeros
pub fn get_unique_id() -> [u8; 8] {
    let mut id = [0u8; 8];
    unsafe {
        FLASH_EEPROM_CMD(RomCmd::GetUniqueId as u8, 0, id.as_mut_ptr(), 0);
    }
    id
}

pub fn get_mac_address() -> [u8; 6] {
    let mut mac = [0u8; 6];
    unsafe {
        FLASH_EEPROM_CMD(RomCmd::GetRomInfo as u8, ROM_CFG_MAC_ADDR, mac.as_mut_ptr(), 0);
    }
    mac
}

pub fn get_boot_info() -> [u8; 12] {
    let mut boot_info = [0u8; 12];
    let mut aligned_boot_info = boot_info.as_ptr() as usize / 4 * 4;
    let new_boot_info = aligned_boot_info as *mut u8;
    unsafe {
        FLASH_EEPROM_CMD(RomCmd::GetRomInfo as u8, ROM_CFG_BOOT_INFO, new_boot_info, 0);
    }
    boot_info
}

pub fn eeprom_read(start_addr: u32, buf: &mut [u8]) -> u32 {
    unsafe { FLASH_EEPROM_CMD(RomCmd::EepromRead as u8, start_addr, buf.as_mut_ptr(), buf.len() as _) }
}

/// @param   StartAddr   - Address of the data to be erased.
/// @param   Length      - Size of data to be erased, in bytes.
/// @return  0-SUCCESS  (!0)-FAILURE
pub fn eeprom_erase(start_addr: u32, len: u32) -> u32 {
    unsafe { FLASH_EEPROM_CMD(RomCmd::EepromErase as u8, start_addr, ptr::null_mut(), len) }
}

pub fn eeprom_write(start_addr: u32, buf: &[u8]) -> u32 {
    unsafe { FLASH_EEPROM_CMD(RomCmd::EepromWrite as u8, start_addr, buf.as_ptr(), buf.len() as _) }
}

/*
pub fn read_flash_rom(start_addr: u32, len: u32) -> &'static [u8] {
    unsafe {
        FLASH_EEPROM_CMD(RomCmd::FlashRomVerify
             as u8, start_addr, TEST_BUF.as_mut_ptr(), len);
        &TEST_BUF[..len as usize]
    }
}
*/
