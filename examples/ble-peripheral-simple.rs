#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::ffi::c_void;
use core::mem::size_of_val;
use core::slice;

use ch32v_rt::highcode;
use ch58x_hal as hal;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::channel::Channel;
use embassy_time::{Delay, Duration, Instant, Ticker, Timer};
use embedded_hal_02::timer::Periodic;
use hal::ble::ffi::*;
use hal::ble::gap::*;
use hal::ble::gatt::gattMsgEvent_t;
use hal::ble::TmosEvent;
use hal::gpio::{AnyPin, Input, Level, Output, OutputDrive, Pin, Pull};
use hal::interrupt::Interrupt;
use hal::prelude::*;
use hal::rtc::Rtc;
use hal::uart::UartTx;
use hal::{ble, peripherals, println};

const fn lo_u16(x: u16) -> u8 {
    (x & 0xff) as u8
}
const fn hi_u16(x: u16) -> u8 {
    (x >> 8) as u8
}

// GAP - SCAN RSP data (max size = 31 bytes)
static mut SCAN_RSP_DATA: &[u8] = &[
    // complete name
    0x12, // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    b'S',
    b'i',
    b'm',
    b'p',
    b'l',
    b'e',
    b' ',
    b'P',
    b'e',
    b'r',
    b'i',
    b'p',
    b'h',
    b'e',
    b'r',
    b'a',
    b'l',
    // Connection interval range
    0x05,
    GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
    lo_u16(80), // units of 1.25ms, 80=100ms
    hi_u16(80),
    lo_u16(800), // units of 1.25ms, 800=1000ms
    hi_u16(800),
    // Tx power level
    0x02, // length of this data
    GAP_ADTYPE_POWER_LEVEL,
    0, // 0dBm
];

const SIMPLEPROFILE_SERV_UUID: u16 = 0xFFE0;

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static mut ADVERT_DATA: &[u8] = &[
    0x02, // length of this data
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
    // https://www.bluetooth.com/specifications/assigned-numbers/
    0x04,                             // length of this data including the data type byte
    GAP_ADTYPE_MANUFACTURER_SPECIFIC, // manufacturer specific advertisement data type
    lo_u16(0x07D7),                   // 0x07D7, Nanjing Qinheng Microelectronics Co., Ltd.
    hi_u16(0x07D7),
    0x01, // remains manufacturer specific data
    // service UUID, to notify central devices what services are included
    // in this peripheral
    0x03,                  // length of this data
    GAP_ADTYPE_16BIT_MORE, // some of the UUID's, but not all
    lo_u16(SIMPLEPROFILE_SERV_UUID),
    hi_u16(SIMPLEPROFILE_SERV_UUID),
];

// GAP GATT Attributes
// len = 21 GAP_DEVICE_NAME_LEN
// max_len = 248
static ATT_DEVICE_NAME: &[u8] = b"ch58x-hal peripheral";

// device info service

static DEV_INFO_SERVICE: gattAttrType_t = gattAttrType_t {
    len: ATT_BT_UUID_SIZE,
    uuid: unsafe { primaryServiceUUID.as_ptr() },
};

// System ID characteristic
const DEVINFO_SYSTEM_ID_LEN: usize = 8;

// The list must start with a Service attribute followed by
// all attributes associated with this Service attribute.
static DEVICE_INFO_TABLE: &[gattAttribute_t] = &[
    // Device Information Service
    gattAttribute_t {
        type_: gattAttrType_t {
            len: ATT_BT_UUID_SIZE,
            uuid: unsafe { primaryServiceUUID.as_ptr() },
        },
        permissions: GATT_PERMIT_READ,
        handle: 0,
        pValue: &DEV_INFO_SERVICE as *const _ as _,
    },
    // System ID Declaration
    gattAttribute_t {
        type_: gattAttrType_t {
            len: ATT_BT_UUID_SIZE,
            uuid: unsafe { characterUUID.as_ptr() },
        },
        permissions: GATT_PERMIT_READ,
        handle: 0,
        pValue: &GATT_PROP_READ as *const _ as _,
    },
    // System ID Value
    gattAttribute_t {
        type_: gattAttrType_t {
            len: ATT_BT_UUID_SIZE,
            uuid: &hal::ble::gatt_uuid::SYSTEM_ID_UUID as *const _ as _,
        },
        permissions: GATT_PERMIT_READ,
        handle: 0,
        pValue: [0, 0, 0, 0, 0, 0, 0, 0].as_ptr() as *const _ as _, // 8 bytes
    },
    // Serial Number String Declaration
    gattAttribute_t {
        type_: gattAttrType_t {
            len: ATT_BT_UUID_SIZE,
            uuid: unsafe { characterUUID.as_ptr() },
        },
        permissions: GATT_PERMIT_READ,
        handle: 0,
        pValue: &GATT_PROP_READ as *const _ as _,
    },
    // Serial Number Value
    gattAttribute_t {
        type_: gattAttrType_t {
            len: ATT_BT_UUID_SIZE,
            uuid: &hal::ble::gatt_uuid::SERIAL_NUMBER_UUID as *const _ as _,
        },
        permissions: GATT_PERMIT_READ,
        handle: 0,
        pValue: b"Serial Number xxxx\0".as_ptr() as *const _ as _,
    },
];

fn peripheral_init() {
    // Setup the GAP Peripheral Role Profile
    unsafe {
        // interval unit 1.25ms
        const MIN_INTERVAL: u16 = 6; // 6*1.25 = 7.5ms
        const MAX_INTERVAL: u16 = 100; // 100*1.25 = 125ms

        // Set the GAP Role Parameters
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, 1, &true as *const _ as _);
        GAPRole_SetParameter(
            GAPROLE_SCAN_RSP_DATA,
            SCAN_RSP_DATA.len() as _,
            SCAN_RSP_DATA.as_ptr() as _,
        );
        GAPRole_SetParameter(GAPROLE_ADVERT_DATA, ADVERT_DATA.len() as _, ADVERT_DATA.as_ptr() as _);
        GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, 2, &MIN_INTERVAL as *const _ as _);
        GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, 2, &MAX_INTERVAL as *const _ as _);
    }

    // Set the GAP Characteristics
    unsafe {
        GGS_SetParameter(
            GGS_DEVICE_NAME_ATT,
            ATT_DEVICE_NAME.len() as _,
            ATT_DEVICE_NAME.as_ptr() as _,
        );
    }

    unsafe {
        // units of 625us, 80=50ms
        const ADVERTISING_INTERVAL: u16 = 80;

        // Set advertising interval
        GAP_SetParamValue(TGAP_DISC_ADV_INT_MIN, ADVERTISING_INTERVAL);
        GAP_SetParamValue(TGAP_DISC_ADV_INT_MAX, ADVERTISING_INTERVAL);

        // Enable scan req notify
        GAP_SetParamValue(TGAP_ADV_SCAN_REQ_NOTIFY, 1);
    }

    // Setup the GAP Bond Manager
    unsafe {
        let passkey: u32 = 0; // passkey "000000"
        let pair_mode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
        let mitm = true;
        let bonding = true;
        let io_cap = GAPBOND_IO_CAP_DISPLAY_ONLY; // display only device
        GAPBondMgr_SetParameter(
            GAPBOND_PERI_DEFAULT_PASSCODE,
            size_of_val(&passkey) as _,
            &passkey as *const _ as _,
        );
        GAPBondMgr_SetParameter(GAPBOND_PERI_PAIRING_MODE, 1, &pair_mode as *const _ as _);
        GAPBondMgr_SetParameter(GAPBOND_PERI_MITM_PROTECTION, 1, &mitm as *const _ as _);
        GAPBondMgr_SetParameter(GAPBOND_PERI_IO_CAPABILITIES, 1, &io_cap as *const _ as _);
        GAPBondMgr_SetParameter(GAPBOND_PERI_BONDING_ENABLED, 1, &bonding as *const _ as _);
    }

    // Initialize GATT attributes
    unsafe {
        GGS_AddService(GATT_ALL_SERVICES); // GAP
        GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes
    }

    unsafe {
        unsafe extern "C" fn on_read_attr(
            connHandle: u16,
            pAttr: *mut gattAttribute_t,
            pValue: *mut u8,
            pLen: *mut u16,
            offset: u16,
            maxLen: u16,
            method: u8,
        ) -> u8 {
            println!("on_read_attr: {:x}", connHandle);

            let raw_uuid = slice::from_raw_parts((*pAttr).type_.uuid, 2);
            let uuid = u16::from_le_bytes([raw_uuid[0], raw_uuid[1]]);

            println!("UUID: 0x{:04x}", uuid);

            return 0;
        }
        static DEV_INFO_CB: gattServiceCBs_t = gattServiceCBs_t {
            pfnReadAttrCB: Some(on_read_attr),
            pfnWriteAttrCB: None,
            pfnAuthorizeAttrCB: None,
        };

        //DevInfo_AddService(); // Device Information Service
        GATTServApp_RegisterService(
            DEVICE_INFO_TABLE.as_ptr() as *const _ as _,
            DEVICE_INFO_TABLE.len() as _,
            GATT_MAX_ENCRYPT_KEY_SIZE,
            &DEV_INFO_CB,
        );

        //SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile
    }

    // Setup the SimpleProfile Characteristic Values
    // SimpleProfile_SetParameter

    // Register receive scan request callback
    unsafe {
        static CB: gapRolesBroadcasterCBs_t = gapRolesBroadcasterCBs_t {
            pfnScanRecv: None,
            pfnStateChange: None,
        };
        GAPRole_BroadcasterSetCB(&CB);
    }
}

#[embassy_executor::task]
async fn read_rssi(conn_handle: u16) {
    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        ticker.next().await;
        unsafe {
            let r = GAPRole_ReadRssiCmd(conn_handle);
            if r.is_err() {
                // normally it's already disconnected, quit
                // println!("!! GAPRole_ReadRssiCmd error: {:?}", r);
                return;
            }
        }
    }
}

pub enum AppEvent {
    Connected(u16),
}

static APP_CHANNEL: Channel<CriticalSectionRawMutex, AppEvent, 3> = Channel::new();

async fn peripheral(spawner: Spawner, task_id: u8, mut subscriber: ble::EventSubscriber) {
    unsafe extern "C" fn on_state_change(new_state: gapRole_States_t, pEvent: *mut gapRoleEvent_t) {
        println!("in on_state_change: {}", new_state);
        let event = &*pEvent;

        match new_state {
            GAPROLE_STARTED => {
                println!("initialized..");
            }
            GAPROLE_ADVERTISING => {
                println!("advertising..");
            }
            GAPROLE_WAITING => {
                if event.gap.opcode == GAP_END_DISCOVERABLE_DONE_EVENT {
                    println!("waiting for advertising..");
                } else if event.gap.opcode == GAP_LINK_TERMINATED_EVENT {
                    // Peripheral_LinkTerminated
                    println!("  disconnected .. reason {:x}", event.linkTerminate.reason);
                    // restart advertising here
                    let mut ret: u32 = 0;
                    GAPRole_GetParameter(GAPROLE_ADVERT_ENABLED, &mut ret as *mut _ as *mut c_void);
                    println!("  GAPROLE_ADVERT_ENABLED: {}", ret);

                    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, 1, &true as *const _ as _);
                } else {
                    println!("unknown event: {}", event.gap.opcode);
                }
            }
            GAPROLE_CONNECTED => {
                // Peripheral_LinkEstablished
                if event.gap.opcode == GAP_LINK_ESTABLISHED_EVENT {
                    println!("connected.. !!");
                    // logic
                    // - SBP_PERIODIC_EVT, period = 1600
                    //   - performPeriodicTask
                    // - SBP_PARAM_UPDATE_EVT, after = 6400
                    //   - GAPRole_PeripheralConnParamUpdateReq
                    // - SBP_READ_RSSI_EVT, period = 3200
                    //GAPRole_ReadRssiCmd(event.linkCmpl.connectionHandle);
                    let _ = APP_CHANNEL.try_send(AppEvent::Connected(event.linkCmpl.connectionHandle));
                }
            }
            GAPROLE_ERROR => {
                println!("error..");
            }
            _ => {
                println!("!!! on_state_change unknown state: {}", new_state);
            }
        }
    }
    unsafe extern "C" fn on_rssi_read(conn_handle: u16, rssi: i8) {
        println!("RSSI -{} dB Conn {:x}", -rssi, conn_handle);
    }
    unsafe extern "C" fn on_param_update(conn_handle: u16, interval: u16, slave_latency: u16, timeout: u16) {
        println!(
            "on_param_update Conn handle: {} inverval: {} timeout: {}",
            conn_handle, interval, timeout
        );
    }

    unsafe {
        static BOND_MGR_CB: gapBondCBs_t = gapBondCBs_t {
            passcodeCB: None,
            pairStateCB: None,
            oobCB: None,
        };

        // peripheralStateNotificationCB

        static APP_CB: gapRolesCBs_t = gapRolesCBs_t {
            pfnStateChange: Some(on_state_change),
            pfnRssiRead: Some(on_rssi_read),
            pfnParamUpdate: Some(on_param_update),
        };
        // Start the Device
        GAPRole_PeripheralStartDevice(task_id, &BOND_MGR_CB, &APP_CB);
    }

    loop {
        match select(subscriber.next_message_pure(), APP_CHANNEL.receive()).await {
            Either::First(event) => {
                handle_tmos_event(&event).await;
            }
            Either::Second(event) => match event {
                AppEvent::Connected(conn_handle) => {
                    spawner.spawn(read_rssi(conn_handle)).unwrap();
                }
            },
        }

        //                     spawner.spawn(read_rssi(event.linkCmpl.connectionHandle)).unwrap();
    }
}

async fn handle_tmos_event(event: &TmosEvent) {
    match event.message_id() {
        TmosEvent::GAP_MSG_EVENT => {
            // Peripheral_ProcessGAPMsg
            let msg = event.0 as *const gapRoleEvent_t;

            let opcode = unsafe { (*msg).gap.opcode };
            match opcode {
                GAP_SCAN_REQUEST_EVENT => {
                    println!(
                        "GAP scan request from {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} ...",
                        (*msg).scanReqEvt.scannerAddr[0],
                        (*msg).scanReqEvt.scannerAddr[1],
                        (*msg).scanReqEvt.scannerAddr[2],
                        (*msg).scanReqEvt.scannerAddr[3],
                        (*msg).scanReqEvt.scannerAddr[4],
                        (*msg).scanReqEvt.scannerAddr[5],
                    );
                }
                GAP_PHY_UPDATE_EVENT => {
                    println!(
                        "GAP phy update Rx:{:x} Tx:{:x}",
                        (*msg).linkPhyUpdate.connRxPHYS,
                        (*msg).linkPhyUpdate.connTxPHYS,
                    );
                }
                GAP_LINK_PARAM_UPDATE_EVENT => {
                    println!(
                        "GAP link param update status: {:x} interval: {:x} latency: {:x} timeout: {:x}",
                        (*msg).linkUpdate.status,
                        (*msg).linkUpdate.connInterval,
                        (*msg).linkUpdate.connLatency,
                        (*msg).linkUpdate.connTimeout,
                    );
                }
                _ => {
                    println!("GAP MSG EVENT: {:p} {:x}", msg, opcode);
                }
            }
        }
        TmosEvent::GATT_MSG_EVENT => {
            let msg = event.0 as *const gattMsgEvent_t;
            let method = unsafe { (*msg).method };
            println!("GATT_MSG_EVENT: {:p} {:x}", msg, method);
        }
        _ => {
            println!("peripheral got event: {:?} id=0x{:02x}", event, event.message_id());
        }
    }
}

#[embassy_executor::main(entry = "ch32v_rt::entry")]
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
    println!("MemFree: {}K", hal::stack_free() / 1024);

    spawner.spawn(blink(p.PA8.degrade())).unwrap();

    // BLE part
    println!("BLE Lib Version: {}", ble::lib_version());

    let mut ble_config = ble::Config::default();
    let (task_id, sub) = hal::ble::init(ble_config).unwrap();
    println!("BLE task id: {}", task_id);

    unsafe {
        let r = GAPRole_PeripheralInit();
        println!("GAPRole_PeripheralInit: {:?}", r);
    }

    peripheral_init();

    // Main_Circulation
    spawner.spawn(tmos_mainloop()).unwrap();

    // jump to app code
    peripheral(spawner, task_id, sub).await;

    loop {}
}

#[highcode]
#[embassy_executor::task]
async fn tmos_mainloop() {
    let mut ticker = Ticker::every(Duration::from_micros(300));
    loop {
        ticker.next().await;
        unsafe {
            TMOS_SystemProcess();
        }
    }
}

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

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    use core::fmt::Write;

    let pa9 = unsafe { peripherals::PA9::steal() };
    let uart1 = unsafe { peripherals::UART1::steal() };
    let mut serial = UartTx::new(uart1, pa9, Default::default()).unwrap();

    let _ = writeln!(&mut serial, "\n\n\n{}", info);

    loop {}
}
