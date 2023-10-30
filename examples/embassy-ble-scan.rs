#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch32v_rt::highcode;
use ch58x_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Instant, Timer};
use hal::ble::ffi::*;
use hal::ble::{get_raw_temperature, MacAddress};
use hal::gpio::{AnyPin, Input, Level, Output, OutputDrive, Pin, Pull};
use hal::interrupt::Interrupt;
use hal::prelude::*;
use hal::rtc::Rtc;
use hal::uart::UartTx;
use hal::{ble, peripherals, println};

// GAP - SCAN RSP data (max size = 31 bytes)
#[rustfmt::skip]
static mut SCAN_RSP_DATA: [u8; 16] = [
    // complete name
    0x0c, // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    0x42, // 'B'
    0x72, // 'r'
    0x6f, // 'o'
    0x61, // 'a'
    0x64, // 'd'
    0x63, // 'c'
    0x61, // 'a'
    0x73, // 's'
    0x74, // 't'
    0x65, // 'e'
    0x72, // 'r'

    // Tx power level
    0x02, // length of this data
    GAP_ADTYPE_POWER_LEVEL,
    0, // 0dBm
];
// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
#[rustfmt::skip]
static mut ADVERT_DATA: [u8; 26] = [
    // Flags; this sets the device to use limited discoverable
    // mode (advertises for 30 seconds at a time) instead of general
    // discoverable mode (advertises indefinitely)
    0x02, // length of this data
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
    // Broadcast of the data

    0x04,                             // length of this data including the data type byte
    GAP_ADTYPE_MANUFACTURER_SPECIFIC, // manufacturer specific advertisement data type
    0xD7,
    0x07, // 0x07D7, Nanjing Qinheng Microelectronics Co., Ltd.
    0x01,

    0x0a,
    GAP_ADTYPE_LOCAL_NAME_SHORT,
    b'c',
    b'h',
    b'5',
    b'8',
    b'x',
    b'-',
    b'h',
    b'a',
    b'l',

    0x02,
    GAP_ADTYPE_POWER_LEVEL,
    6, // 6dBm

    0x03,
    GAP_ADTYPE_APPEARANCE,
    0x00, // 0x0200 Generic tag
    0x02,
];

#[embassy_executor::task]
async fn blink(pin: AnyPin) {
    let mut led = Output::new(pin, Level::Low, OutputDrive::_5mA);

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(150)).await;
        led.set_low();
        Timer::after(Duration::from_millis(150)).await;
    }
}

#[embassy_executor::task]
async fn calibrate_ble_reg(interval_ms: u32) {
    loop {
        unsafe {
            BLE_RegInit();
        }
        println!("BLE_RegInit done");
        Timer::after(Duration::from_millis(interval_ms as _)).await;
    }
}

unsafe extern "C" fn broadcaster_callback(new_state: u32) {
    println!("broadcast state: {}", new_state);
    match new_state {
        GAPROLE_STARTED => {
            println!("initialized..");
        }
        GAPROLE_ADVERTISING => {
            println!("advertising..");
        }
        GAPROLE_WAITING => {
            println!("waiting for advertising..");
        }
        GAPROLE_ERROR => {
            println!("error..");
        }
        _ => {
            println!("!!! unknown state: {}", new_state);
        }
    }
}

// Broadcaster_ProcessEvent
// No tmos msg handling logic here, so maybe I can rewrite this to embassy task.
#[embassy_executor::task]
async fn broadcaster() {
    static CALLBACKS: gapRolesBroadcasterCBs_t = gapRolesBroadcasterCBs_t {
        pfnStateChange: Some(broadcaster_callback),
        pfnScanRecv: None,
    };

    unsafe {
        println!("set up callback=> {:?}", CALLBACKS);
        let r = GAPRole_BroadcasterStartDevice(&CALLBACKS);
        println!("GAPRole_BroadcasterStartDevice: {:?}", r);
    }
}
unsafe extern "C" fn observer_event_callback(event: &gapRoleEvent_t) {
    const DEFAULT_DISCOVERY_MODE: u8 = DEVDISC_MODE_ALL;
    const DEFAULT_DISCOVERY_ACTIVE_SCAN: u8 = 0; // false
    const DEFAULT_DISCOVERY_WHITE_LIST: u8 = 0;

    // println!("observer_event_callback: {:?}", event.gap);
    println!("event opcode: {:?}", event.gap.opcode);
    match event.gap.opcode {
        GAP_DEVICE_INIT_DONE_EVENT => {
            println!("Discovering...");
            GAPRole_ObserverStartDiscovery(
                DEFAULT_DISCOVERY_MODE,
                DEFAULT_DISCOVERY_ACTIVE_SCAN,
                DEFAULT_DISCOVERY_WHITE_LIST,
            );
        }
        GAP_DEVICE_DISCOVERY_EVENT => {
            println!("Complete");
        }
        GAP_DEVICE_INFO_EVENT => {
            let event = event.deviceInfo;
            println!("Device => {}, rssi={}", MacAddress::from_raw(event.addr), event.rssi);
        }
        _ => {
            println!("unknown event {}", event.gap.opcode);
        }
    }
}

#[embassy_executor::task]
async fn observer() {
    static CALLBACKS: gapRoleObserverCB_t = gapRoleObserverCB_t {
        eventCB: Some(observer_event_callback),
    };
    unsafe {
        GAPRole_ObserverStartDevice(&CALLBACKS);
    }
}

#[embassy_executor::main(entry = "ch32v_rt::entry")]
#[highcode]
async fn main(spawner: Spawner) -> ! {
    use hal::ble::ffi::*;

    let mut config = hal::Config::default();
    config.clock.use_pll_60mhz().enable_lse();
    let p = hal::init(config);
    hal::embassy::init();

    let uart = UartTx::new(p.UART1, p.PA9, Default::default()).unwrap();
    unsafe {
        hal::set_default_serial(uart);
    }

    let boot_btn = Input::new(p.PB22, Pull::Up);

    let rtc = Rtc::new(p.RTC);

    println!();
    println!("Hello World from ch58x-hal!");
    println!(
        r#"
    ______          __
   / ____/___ ___  / /_  ____ _____________  __
  / __/ / __ `__ \/ __ \/ __ `/ ___/ ___/ / / /
 / /___/ / / / / / /_/ / /_/ (__  |__  ) /_/ /
/_____/_/ /_/ /_/_.___/\__,_/____/____/\__, /
                                      /____/   on CH582"#
    );
    println!("System Clocks: {}", hal::sysctl::clocks().hclk);
    println!("ChipID: 0x{:02x}", hal::signature::get_chip_id());
    println!("RTC datetime: {}", rtc.now());

    let r = ble::init(ble::Config {
        mac_addr: [0x22, 0x33, 0x44, 0x55, 0x66, 0x77],
    });
    println!("ble init: {:?}", r);
    println!("MemFree: {}K", hal::stack_free() / 1024);

    // start ble reg calibrate loop
    spawner.spawn(calibrate_ble_reg(120_000)).unwrap();

    unsafe {
        GAPRole_ObserverInit();
    }

    // Observer_Init
    unsafe {
        // 4800 * 0.625us = 3ms
        GAP_SetParamValue(TGAP_DISC_SCAN, 4800);
        GAP_SetParamValue(TGAP_DISC_SCAN_PHY, GAP_PHY_BIT_LE_1M);

        // Observer_ProcessEvent
    }

    spawner.spawn(observer()).unwrap();

    spawner.spawn(blink(p.PA8.degrade())).unwrap();

    // Main_Circulation
    loop {
        Timer::after(Duration::from_micros(300)).await;
        unsafe {
            hal::interrupt::SysTick::pend();
            TMOS_SystemProcess();
            hal::interrupt::SysTick::unpend();
        }

        // println!("tick");
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    use core::fmt::Write;

    let pa9 = unsafe { peripherals::PA9::steal() };
    let uart1 = unsafe { peripherals::UART1::steal() };
    let mut serial = UartTx::new(uart1, pa9, Default::default()).unwrap();

    let _ = writeln!(&mut serial, "\n\n\n{}", info);

    loop {}
}
