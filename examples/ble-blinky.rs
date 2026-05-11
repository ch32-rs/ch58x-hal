//! HeartRate Peripheral, HRS

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use core::mem::{size_of_val, MaybeUninit};
use core::sync::atomic::{AtomicBool, Ordering};
use core::{ptr, slice};

use qingke_rt::highcode;
use ch58x_hal as hal;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Ticker, Timer};
use hal::ble::ffi::*;
use hal::ble::gap::*;
use hal::ble::gatt::*;
use hal::ble::gattservapp::*;
use hal::ble::{gatt_uuid, TmosEvent};
use hal::gpio::{AnyPin, Level, Output, OutputDrive, Pin};
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
    b'B',
    b'l',
    b'i',
    b'n',
    b'k',
    b'y',
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

// const SIMPLEPROFILE_SERV_UUID: u16 = 0xFFE0;

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
#[rustfmt::skip]
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

    // advertised service
    0x03,                  // length of this data
    GAP_ADTYPE_16BIT_MORE, // some of the UUID's, but not all
    lo_u16(BLINKY_SERV_UUID),
    hi_u16(BLINKY_SERV_UUID),
];

// GAP GATT Attributes
// len = 21 GAP_DEVICE_NAME_LEN
// max_len = 248
static ATT_DEVICE_NAME: &[u8] = b"ch58x-hal peripheral";

// System ID characteristic
const DEVINFO_SYSTEM_ID_LEN: usize = 8;

static mut SYSTEM_ID: [u8; 8] = [0u8; 8];
// The list must start with a Service attribute followed by
// all attributes associated with this Service attribute.
// Must use static mut fixed sized array, as it will be changed by Service to assign handles.
static mut DEVICE_INFO_TABLE: [GattAttribute; 7] =
    [
        // Device Information Service
        GattAttribute {
            type_: GattAttrType {
                len: ATT_BT_UUID_SIZE,
                uuid: unsafe { gatt_uuid::primaryServiceUUID.as_ptr() },
            },
            permissions: GATT_PERMIT_READ,
            handle: 0,
            // The first must be a Service attribute
            value: &GattAttrType {
                len: ATT_BT_UUID_SIZE,
                uuid: &gatt_uuid::DEVINFO_SERV_UUID as *const _ as _,
            } as *const _ as _,
        },
        // System ID Declaration
        GattAttribute {
            type_: GattAttrType {
                len: ATT_BT_UUID_SIZE,
                uuid: unsafe { gatt_uuid::characterUUID.as_ptr() },
            },
            permissions: GATT_PERMIT_READ,
            handle: 0,
            value: &GATT_PROP_READ as *const _ as _,
        },
        // System ID Value
        GattAttribute {
            type_: GattAttrType {
                len: ATT_BT_UUID_SIZE,
                uuid: &gatt_uuid::SYSTEM_ID_UUID as *const _ as _,
            },
            permissions: GATT_PERMIT_READ,
            handle: 0,
            value: unsafe { SYSTEM_ID.as_ptr() },
        },
        // Serial Number String Declaration
        GattAttribute {
            type_: GattAttrType {
                len: ATT_BT_UUID_SIZE,
                uuid: unsafe { gatt_uuid::characterUUID.as_ptr() },
            },
            permissions: GATT_PERMIT_READ,
            handle: 0,
            value: &GATT_PROP_READ as *const _ as _,
        },
        // Serial Number Value
        GattAttribute {
            type_: GattAttrType {
                len: ATT_BT_UUID_SIZE,
                uuid: &gatt_uuid::SERIAL_NUMBER_UUID as *const _ as _,
            },
            permissions: GATT_PERMIT_READ,
            handle: 0,
            value: ptr::null(),
        },
        // Temperature
        GattAttribute {
            type_: GattAttrType {
                len: ATT_BT_UUID_SIZE,
                uuid: unsafe { gatt_uuid::characterUUID.as_ptr() },
            },
            permissions: GATT_PERMIT_READ,
            handle: 0,
            value: &GATT_PROP_READ as *const _ as _,
        },
        // Serial Number Value
        GattAttribute {
            type_: GattAttrType {
                len: ATT_BT_UUID_SIZE,
                uuid: &gatt_uuid::TEMP_UUID as *const _ as _,
            },
            permissions: GATT_PERMIT_READ,
            handle: 0,
            value: ptr::null(),
        },
    ];

#[inline]
unsafe fn devinfo_init() {
    // DevInfo_AddService
    unsafe {
        unsafe extern "C" fn dev_info_on_read_attr(
            _conn_handle: u16,
            attr: *mut GattAttribute,
            value: *mut u8,
            plen: *mut u16,
            _offset: u16,
            max_len: u16,
            _method: u8,
        ) -> u8 {
            let raw_uuid = slice::from_raw_parts((*attr).type_.uuid, 2);
            let uuid = u16::from_le_bytes([raw_uuid[0], raw_uuid[1]]);
            println!("! on_read_attr UUID: 0x{:04x}", uuid);

            match uuid {
                gatt_uuid::SYSTEM_ID_UUID => {
                    *plen = DEVINFO_SYSTEM_ID_LEN as _;
                    ptr::copy(SYSTEM_ID.as_ptr(), value, DEVINFO_SYSTEM_ID_LEN);
                }
                gatt_uuid::SERIAL_NUMBER_UUID => {
                    let out = b"No. 9527";
                    *plen = out.len() as _;
                    core::ptr::copy(out.as_ptr(), value, out.len());
                }
                gatt_uuid::TEMP_UUID => {
                    println!("temp uuid {:04x} {:p} {}", uuid, value, max_len);
                    let val: i16 = 32_00; // 0.01 degC
                    *plen = size_of_val(&val) as _;
                    core::ptr::copy(&val as *const _ as _, value, *plen as _);
                }
                _ => {
                    return ATT_ERR_ATTR_NOT_FOUND;
                }
            }

            return 0;
        }
        static DEV_INFO_SERVICE_CB: gattServiceCBs_t = gattServiceCBs_t {
            pfnReadAttrCB: Some(dev_info_on_read_attr),
            pfnWriteAttrCB: None,
            pfnAuthorizeAttrCB: None,
        };
        // DevInfo_AddService(); // Device Information Service
        // might fail, must check
        GATTServApp::register_service(
            &mut DEVICE_INFO_TABLE[..],
            GATT_MAX_ENCRYPT_KEY_SIZE,
            &DEV_INFO_SERVICE_CB,
        )
        .unwrap();
    }
}

/// GAP Role init
unsafe fn common_init() {
    // Setup the GAP Peripheral Role Profile
    {
        // Set the GAP Role Parameters
        let _ = GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, 1, &true as *const _ as _);
        let _ = GAPRole_SetParameter(
            GAPROLE_SCAN_RSP_DATA,
            SCAN_RSP_DATA.len() as _,
            SCAN_RSP_DATA.as_ptr() as _,
        );
        let _ = GAPRole_SetParameter(GAPROLE_ADVERT_DATA, ADVERT_DATA.len() as _, ADVERT_DATA.as_ptr() as _);
    }

    // Set the GAP Characteristics
    let _ = GGS_SetParameter(
        GGS_DEVICE_NAME_ATT,
        ATT_DEVICE_NAME.len() as _,
        ATT_DEVICE_NAME.as_ptr() as _,
    );

    // Setup the GAP Bond Manager
    {
        let passkey: u32 = 0; // passkey "000000"
        let pair_mode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
        let mitm = false;
        let io_cap = GAPBOND_IO_CAP_DISPLAY_ONLY;
        let bonding = true;
        let _ = GAPBondMgr_SetParameter(
            GAPBOND_PERI_DEFAULT_PASSCODE,
            size_of_val(&passkey) as _,
            &passkey as *const _ as _,
        );
        let _ = GAPBondMgr_SetParameter(GAPBOND_PERI_PAIRING_MODE, 1, &pair_mode as *const _ as _);
        let _ = GAPBondMgr_SetParameter(GAPBOND_PERI_MITM_PROTECTION, 1, &mitm as *const _ as _);
        let _ = GAPBondMgr_SetParameter(GAPBOND_PERI_IO_CAPABILITIES, 1, &io_cap as *const _ as _);
        let _ = GAPBondMgr_SetParameter(GAPBOND_PERI_BONDING_ENABLED, 1, &bonding as *const _ as _);
    }

    // Initialize GATT attributes
    {
        let _ = GGS_AddService(GATT_ALL_SERVICES).unwrap(); // GAP
        let _ = GATTServApp::add_service(GATT_ALL_SERVICES).unwrap(); // GATT attributes
    }

    // Add other service
}

const BLINKY_SERV_UUID: u16 = 0xFFE0;
const BLINKY_DATA_UUID: u16 = 0xFFE1;
const BLINKY_CONF_UUID: u16 = 0xFFE2;
const BLINKY_CMD_UUID: u16 = 0xFFE3;

static mut BLINKY_CLIENT_CHARCFG: [gattCharCfg_t; 4] = unsafe { core::mem::zeroed() };

static mut BLINKY_ATTR_TABLE: [GattAttribute; 6] =
    [
        // Blinky Service
        GattAttribute {
            type_: GattAttrType::PRIMARY_SERVICE,
            permissions: GATT_PERMIT_READ,
            handle: 0,
            value: &GattAttrType {
                len: ATT_BT_UUID_SIZE,
                uuid: &BLINKY_SERV_UUID as *const _ as _,
            } as *const _ as _,
        },
        // Blinky Data Declaration and Value
        GattAttribute {
            type_: GattAttrType::CHARACTERISTIC,
            permissions: GATT_PERMIT_READ,
            handle: 0,
            value: &(GATT_PROP_NOTIFY | GATT_PROP_READ) as *const _ as _,
        },
        GattAttribute {
            type_: GattAttrType::new_u16(&BLINKY_DATA_UUID),
            permissions: GATT_PERMIT_READ,
            handle: 0,
            value: ptr::null(), // this will be filled in read callback
        },
        // Blinky client config
        GattAttribute {
            type_: GattAttrType::CLIENT_CHAR_CFG,
            permissions: GATT_PERMIT_READ | GATT_PERMIT_WRITE,
            handle: 0,
            value: unsafe { BLINKY_CLIENT_CHARCFG.as_ptr() as _ },
        },
        // Command
        GattAttribute {
            type_: GattAttrType::CHARACTERISTIC,
            permissions: GATT_PERMIT_READ,
            handle: 0,
            value: &GATT_PROP_WRITE as *const _ as _,
        },
        GattAttribute {
            type_: GattAttrType::new_u16(&BLINKY_CMD_UUID),
            permissions: GATT_PERMIT_WRITE,
            handle: 0,
            value: ptr::null(),
        },
    ];

unsafe fn blinky_init() {
    unsafe extern "C" fn blinky_on_read_attr(
        _conn_handle: u16,
        attr: *mut GattAttribute,
        value: *mut u8,
        plen: *mut u16,
        offset: u16,
        max_len: u16,
        _method: u8,
    ) -> u8 {
        // Make sure it's not a blob operation (no attributes in the profile are long)
        if (offset > 0) {
            return ATT_ERR_ATTR_NOT_LONG;
        }

        let uuid = *((*attr).type_.uuid as *const u16);
        println!("! on_read_attr UUID: 0x{:04x}", uuid);

        match uuid {
            BLINKY_DATA_UUID => {
                let on = BLINKY_ON.load(Ordering::Relaxed);
                let val: u8 = if on { 0x01 } else { 0x00 };
                *plen = size_of_val(&val) as _;
                core::ptr::copy(&val as *const _ as _, value, *plen as _);
            }
            _ => {
                return ATT_ERR_ATTR_NOT_FOUND;
            }
        }

        return 0;
    }
    unsafe extern "C" fn blinky_on_write_attr(
        conn_handle: u16,
        attr: *mut GattAttribute,
        value: *mut u8,
        len: u16,
        offset: u16,
        method: u8,
    ) -> u8 {
        let uuid = *((*attr).type_.uuid as *const u16);
        println!("! on_write_attr UUID: 0x{:04x}", uuid);

        if uuid == BLINKY_CMD_UUID {
            let cmd = *value;
            println!("! on_write_attr cmd: 0x{:02x}", cmd);
            if cmd == 0x01 {
                BLINKY_ON.store(true, Ordering::Relaxed);
            } else if cmd == 0x00 {
                BLINKY_ON.store(false, Ordering::Relaxed);
            }
        } else if uuid == BLINKY_CONF_UUID {
            // sub to notrification
            //  let status = GATTServApp::process_ccc_write_req(conn_handle, attr, value, len, offset, GATT_CLIENT_CFG_NOTIFY);
            // if status.is_ok() {
            //    println!("! on_write_attr sub");
            //    let val = slice::from_raw_parts(value, len as usize);
            //    println!("! on_write_attr sub value {:?}", val);
            // }
            //APP_CHANNEL.try_send(AppEvent::BlinkySubscribed(conn_handle));
            //println!("! on_write_attr sub");
        } else if uuid == gatt_uuid::GATT_CLIENT_CHAR_CFG_UUID {
            // client char cfg
            let status =
                GATTServApp::process_ccc_write_req(conn_handle, attr, value, len, offset, GATT_CLIENT_CFG_NOTIFY);
            if status.is_ok() {
                println!("! on_write_attr sub");
                let val = slice::from_raw_parts(value, len as usize);
                println!("! on_write_attr sub value {:?}", val);
                if val == &[0x01, 0x00] {
                    APP_CHANNEL.try_send(AppEvent::BlinkySubscribed(conn_handle));
                } else {
                    APP_CHANNEL.try_send(AppEvent::BlinkyUnsubscribed(conn_handle));
                }
            }
        } else {
            return ATT_ERR_ATTR_NOT_FOUND;
        }

        return 0;
    }

    static BLINKY_SERVICE_CB: gattServiceCBs_t = gattServiceCBs_t {
        pfnReadAttrCB: Some(blinky_on_read_attr),
        pfnWriteAttrCB: Some(blinky_on_write_attr),
        pfnAuthorizeAttrCB: None,
    };

    // Initialize Client Characteristic Configuration attributes
    GATTServApp::init_char_cfg(INVALID_CONNHANDLE, BLINKY_CLIENT_CHARCFG.as_mut_ptr());

    GATTServApp::register_service(
        &mut BLINKY_ATTR_TABLE[..],
        GATT_MAX_ENCRYPT_KEY_SIZE,
        &BLINKY_SERVICE_CB,
    )
    .unwrap();
}

// App logic

pub enum AppEvent {
    Connected(u16),
    Disconnected(u16),
    BlinkySubscribed(u16),
    BlinkyUnsubscribed(u16),
}

static APP_CHANNEL: Channel<CriticalSectionRawMutex, AppEvent, 3> = Channel::new();

/// Default desired minimum connection interval (units of 1.25ms)
const DEFAULT_DESIRED_MIN_CONN_INTERVAL: u16 = 20;
/// Default desired maximum connection interval (units of 1.25ms)
const DEFAULT_DESIRED_MAX_CONN_INTERVAL: u16 = 160;
/// Default desired slave latency to use if parameter update request
const DEFAULT_DESIRED_SLAVE_LATENCY: u16 = 1;
/// Default supervision timeout value (units of 10ms)
const DEFAULT_DESIRED_CONN_TIMEOUT: u16 = 1000;

async fn peripheral(spawner: Spawner, task_id: u8, mut subscriber: ble::EventSubscriber) -> ! {
    // Profile State Change Callbacks
    unsafe extern "C" fn on_gap_state_change(new_state: gapRole_States_t, event: *mut gapRoleEvent_t) {
        println!("in on_gap_state_change: {}", new_state);
        let event = &*event;

        // state machine, requires last state
        static mut LAST_STATE: gapRole_States_t = GAPROLE_INIT;

        // time units 625us
        const DEFAULT_FAST_ADV_INTERVAL: u16 = 32;
        const DEFAULT_FAST_ADV_DURATION: u16 = 30000;

        const DEFAULT_SLOW_ADV_INTERVAL: u16 = 1600;
        const DEFAULT_SLOW_ADV_DURATION: u16 = 0; // continuous

        static mut CONN_HANDLE: u16 = INVALID_CONNHANDLE;

        match new_state {
            GAPROLE_CONNECTED => {
                // Peripheral_LinkEstablished
                if event.gap.opcode == GAP_LINK_ESTABLISHED_EVENT {
                    println!("connected.. !!");
                    CONN_HANDLE = event.linkCmpl.connectionHandle;

                    let _ = APP_CHANNEL.try_send(AppEvent::Connected(CONN_HANDLE));
                }
            }
            // if disconnected
            _ if LAST_STATE == GAPROLE_CONNECTED && new_state != GAPROLE_CONNECTED => {
                // link loss -- use fast advertising
                let _ = GAP_SetParamValue(TGAP_DISC_ADV_INT_MIN, DEFAULT_FAST_ADV_INTERVAL);
                let _ = GAP_SetParamValue(TGAP_DISC_ADV_INT_MAX, DEFAULT_FAST_ADV_INTERVAL);
                let _ = GAP_SetParamValue(TGAP_GEN_DISC_ADV_MIN, DEFAULT_FAST_ADV_DURATION);

                let _ = APP_CHANNEL.try_send(AppEvent::Disconnected(CONN_HANDLE));

                // Enable advertising
                let _ = GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, 1, &true as *const _ as _);
            }
            // if advertising stopped
            GAPROLE_WAITING if LAST_STATE == GAPROLE_ADVERTISING => {
                // if fast advertising switch to slow
                if GAP_GetParamValue(TGAP_DISC_ADV_INT_MIN) == DEFAULT_FAST_ADV_INTERVAL {
                    let _ = GAP_SetParamValue(TGAP_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL);
                    let _ = GAP_SetParamValue(TGAP_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL);
                    let _ = GAP_SetParamValue(TGAP_GEN_DISC_ADV_MIN, DEFAULT_SLOW_ADV_DURATION);
                    let _ = GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, 1, &true as *const _ as _);
                }
            }
            // if started
            GAPROLE_STARTED => {
                println!("initialized..");
                let mut system_id = [0u8; 8]; // DEVINFO_SYSTEM_ID_LEN
                GAPRole_GetParameter(GAPROLE_BD_ADDR, system_id.as_mut_ptr() as _).unwrap();

                // shift three bytes up
                system_id[7] = system_id[5];
                system_id[6] = system_id[4];
                system_id[5] = system_id[3];

                // set middle bytes to zero
                system_id[4] = 0;
                system_id[3] = 0;

                ptr::copy(system_id.as_ptr(), SYSTEM_ID.as_mut_ptr(), 8);
            }
            GAPROLE_ADVERTISING => {} // now advertising
            _ => {
                println!("!!! on_state_change unhandled state: {}", new_state);
            }
        }

        LAST_STATE = new_state;
    }

    // Deivce start
    unsafe {
        static BOND_CB: gapBondCBs_t = gapBondCBs_t {
            passcodeCB: None,
            pairStateCB: None,
            oobCB: None,
        };
        // peripheralStateNotificationCB
        static APP_CB: gapRolesCBs_t = gapRolesCBs_t {
            pfnStateChange: Some(on_gap_state_change),
            pfnRssiRead: None,
            pfnParamUpdate: None,
        };
        // Start the Device
        let r = GAPRole_PeripheralStartDevice(task_id, &BOND_CB, &APP_CB);
        println!("Start device {:?}", r);
    }

    loop {
        match select(subscriber.next_message_pure(), APP_CHANNEL.receive()).await {
            Either::First(event) => {
                handle_tmos_event(&event).await;
            }
            Either::Second(event) => match event {
                AppEvent::Connected(conn_handle) => unsafe {
                    // 1600 * 625 us
                    Timer::after(Duration::from_secs(1)).await; // FIXME: spawn handler

                    GAPRole_PeripheralConnParamUpdateReq(
                        conn_handle,
                        DEFAULT_DESIRED_MIN_CONN_INTERVAL,
                        DEFAULT_DESIRED_MAX_CONN_INTERVAL,
                        DEFAULT_DESIRED_SLAVE_LATENCY,
                        DEFAULT_DESIRED_CONN_TIMEOUT,
                        task_id,
                    )
                    .unwrap();
                },
                AppEvent::Disconnected(conn_handle) => unsafe {
                    GATTServApp::init_char_cfg(conn_handle, BLINKY_CLIENT_CHARCFG.as_mut_ptr());
                },
                AppEvent::BlinkySubscribed(conn_handle) => unsafe {
                    spawner.spawn(blinky_notification(conn_handle)).unwrap();
                },
                _ => {
                    // other event. just broadcast
                }
            },
        }
    }
}

#[embassy_executor::task]
async fn blinky_notification(conn_handle: u16) {
    let mut ticker = Ticker::every(Duration::from_millis(1000));

    static mut NOTIFY_MSG: gattMsg_t = gattMsg_t {
        handleValueNoti: attHandleValueNoti_t {
            handle: 0,
            len: 2,
            pValue: ptr::null_mut(),
        },
    };
    loop {
        match select(ticker.next(), APP_CHANNEL.receive()).await {
            Either::First(_) => unsafe {
                let val = GATTServApp::read_char_cfg(conn_handle, BLINKY_CLIENT_CHARCFG.as_ptr());
                if val == 0x01 {
                    // notification is no
                    let on = BLINKY_ON.load(Ordering::Relaxed);
                    let val: u8 = if on { 0x01 } else { 0x00 };
                    // let mut msg = gattMsg_t::alloc_handle_value_notification(conn_handle, 2);

                    unsafe {
                        NOTIFY_MSG.handleValueNoti.pValue =
                            GATT_bm_alloc(0, ATT_HANDLE_VALUE_NOTI, 2, ptr::null_mut(), 0) as _;
                        NOTIFY_MSG.handleValueNoti.handle = BLINKY_ATTR_TABLE[2].handle;
                        NOTIFY_MSG.handleValueNoti.len = 2;

                        core::ptr::copy(&val as *const _ as _, NOTIFY_MSG.handleValueNoti.pValue, 2);
                        println!("!! handle {}", BLINKY_ATTR_TABLE[2].handle);

                        let rc = GATT_Notification(conn_handle, &NOTIFY_MSG.handleValueNoti, 0);
                        println!("!! notify rc {:?}", rc);
                    }
                }
            },
            Either::Second(AppEvent::Disconnected(_)) | Either::Second(AppEvent::BlinkyUnsubscribed(_)) => {
                println!("disconnect, stop notification");
                return;
            }
            _ => (),
        }
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
    println!("MemFree: {}K", hal::stack_free() / 1024);

    spawner.spawn(blink(p.PA8.degrade())).unwrap();

    // BLE part
    println!("BLE Lib Version: {}", ble::lib_version());

    let mut ble_config = ble::Config::default();
    ble_config.pa_config = None;
    ble_config.mac_addr = [0xca, 0xfe, 0xba, 0xbe, 0x01, 0x01].into();
    let (task_id, sub) = hal::ble::init(ble_config).unwrap();
    println!("BLE hal task id: {}", task_id);

    let _ = GAPRole::peripheral_init().unwrap();

    unsafe {
        common_init();

        devinfo_init();

        blinky_init();
    }

    // Main_Circulation
    spawner.spawn(tmos_mainloop()).unwrap();

    // Application code
    peripheral(spawner, task_id, sub).await
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

static BLINKY_ON: AtomicBool = AtomicBool::new(true);

#[embassy_executor::task]
async fn blink(pin: AnyPin) {
    let mut led = Output::new(pin, Level::Low, OutputDrive::_5mA);

    loop {
        if BLINKY_ON.load(Ordering::Relaxed) {
            led.toggle();
        }
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
