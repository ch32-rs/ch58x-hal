#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use qingke_rt::highcode;
use ch58x_hal as hal;
use ch58x_hal::ble::gap::*;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::ble::ffi::*;
use hal::gpio::{AnyPin, Level, Output, OutputDrive, Pin};
use hal::interrupt::Interrupt;
use hal::rtc::Rtc;
use hal::uart::UartTx;
use hal::{ble, peripherals, println};

// GAP - SCAN RSP data (max size = 31 bytes)
static mut SCAN_RSP_DATA: [u8; 16] =
    [
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
static mut ADVERT_DATA: [u8; 22] =
    [
        0x02, // length of this data
        GAP_ADTYPE_FLAGS,
        GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
        // https://www.bluetooth.com/specifications/assigned-numbers/
        0x04,                             // length of this data including the data type byte
        GAP_ADTYPE_MANUFACTURER_SPECIFIC, // manufacturer specific advertisement data type
        0xD7,
        0x07, // 0x07D7, Nanjing Qinheng Microelectronics Co., Ltd.
        0x01,
        0x0a, // len
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
        0x02, // len
        GAP_ADTYPE_POWER_LEVEL,
        0, // 0dBm
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

#[highcode]
async fn mainloop() -> ! {
    loop {
        Timer::after(Duration::from_micros(300)).await;
        unsafe {
            TMOS_SystemProcess();
        }
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
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

    println!("BLE Lib Version: {}", ble::lib_version());

    let (task_id, _) = hal::ble::init(Default::default()).unwrap();
    println!("init BLE task id: {}", task_id);

    unsafe {
        println!("Gen Addr: {:08x}", BLE_AccessAddressGenerate());
        let r = GAPRole_BroadcasterInit();
        println!("GAPRole_BroadcasterInit: {:?}", r);
    }

    // Broadcaster_Init();
    unsafe {
        // Setup the GAP Broadcaster Role Profile
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, 1, &true as *const _ as _).unwrap();
        GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, 1, &0x03 as *const _ as _).unwrap();
        GAPRole_SetParameter(
            GAPROLE_SCAN_RSP_DATA,
            SCAN_RSP_DATA.len() as _,
            SCAN_RSP_DATA.as_mut_ptr() as _,
        )
        .unwrap();
        GAPRole_SetParameter(
            GAPROLE_ADVERT_DATA,
            ADVERT_DATA.len() as _,
            ADVERT_DATA.as_mut_ptr() as _,
        )
        .unwrap();
    }

    // setting advertising interval
    unsafe {
        let adv_interval = 160; // in 0.625ms, 100ms

        GAP_SetParamValue(TGAP_DISC_ADV_INT_MIN, adv_interval).unwrap();
        GAP_SetParamValue(TGAP_DISC_ADV_INT_MAX, adv_interval).unwrap();
    }

    spawner.spawn(broadcaster()).unwrap();

    spawner.spawn(blink(p.PA8.degrade())).unwrap();

    // Main_Circulation
    mainloop().await
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
