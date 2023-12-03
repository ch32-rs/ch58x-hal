//! GAP layer interface
//!
#![allow(non_snake_case, non_camel_case_types)]

use super::ffi::{bStatus_t, tmos_event_hdr_t};

// event.gap.opcode
/// Sent when the Device Initialization is complete.
pub const GAP_DEVICE_INIT_DONE_EVENT: u8 = 0x00;
/// Sent when the Device Discovery Process is complete.
pub const GAP_DEVICE_DISCOVERY_EVENT: u8 = 0x01;
/// Sent when the Advertising Data or SCAN_RSP Data has been updated.
pub const GAP_ADV_DATA_UPDATE_DONE_EVENT: u8 = 0x02;
/// Sent when the Make Discoverable Request is complete.
pub const GAP_MAKE_DISCOVERABLE_DONE_EVENT: u8 = 0x03;
/// Sent when the Advertising has ended.
pub const GAP_END_DISCOVERABLE_DONE_EVENT: u8 = 0x04;
/// Sent when the Establish Link Request is complete.
pub const GAP_LINK_ESTABLISHED_EVENT: u8 = 0x05;
/// Sent when a connection was terminated.
pub const GAP_LINK_TERMINATED_EVENT: u8 = 0x06;
/// Sent when an Update Parameters Event is received.
pub const GAP_LINK_PARAM_UPDATE_EVENT: u8 = 0x07;
/// Sent when a random address was changed.
pub const GAP_RANDOM_ADDR_CHANGED_EVENT: u8 = 0x08;
/// Sent when the device's signature counter is updated.
pub const GAP_SIGNATURE_UPDATED_EVENT: u8 = 0x09;
/// Sent when the Authentication (pairing) process is complete.
pub const GAP_AUTHENTICATION_COMPLETE_EVENT: u8 = 0x0A;
/// Sent when a Passkey is needed. This is part of the pairing process.
pub const GAP_PASSKEY_NEEDED_EVENT: u8 = 0x0B;
/// Sent when a Slave Security Request is received.
pub const GAP_SLAVE_REQUESTED_SECURITY_EVENT: u8 = 0x0C;
/// Sent during the Device Discovery Process when a device is discovered.
pub const GAP_DEVICE_INFO_EVENT: u8 = 0x0D;
/// Sent when the bonding process is complete.
pub const GAP_BOND_COMPLETE_EVENT: u8 = 0x0E;
/// Sent when an unexpected Pairing Request is received.
pub const GAP_PAIRING_REQ_EVENT: u8 = 0x0F;
/// Sent when a direct Advertising Data is received.
pub const GAP_DIRECT_DEVICE_INFO_EVENT: u8 = 0x10;
/// Sent when a PHY Update Event is received.
pub const GAP_PHY_UPDATE_EVENT: u8 = 0x11;
/// Sent when a Extended Advertising Data is received.
pub const GAP_EXT_ADV_DEVICE_INFO_EVENT: u8 = 0x12;
/// Sent when the Set Periodic Advertising enable is complete.
pub const GAP_MAKE_PERIODIC_ADV_DONE_EVENT: u8 = 0x13;
/// Sent when the Set Periodic Advertising disable is complete.
pub const GAP_END_PERIODIC_ADV_DONE_EVENT: u8 = 0x14;
/// Sent when a Periodic Advertising Sync Establish is complete.
pub const GAP_SYNC_ESTABLISHED_EVENT: u8 = 0x15;
/// Sent when a Periodic Advertising Data is received.
pub const GAP_PERIODIC_ADV_DEVICE_INFO_EVENT: u8 = 0x16;
/// Sent when a Periodic Advertising Sync was lost.
pub const GAP_SYNC_LOST_EVENT: u8 = 0x17;
/// Sent when a SCAN_REQ PDU or an AUX_SCAN_REQ PDU has been received by the advertiser.
pub const GAP_SCAN_REQUEST_EVENT: u8 = 0x19;
/// resv
pub const GAP_OOB_NEEDED_EVENT: u8 = 0x1A;
/// Sent when the Set Connectionless CTE Transmit enable is complete.
pub const GAP_MAKE_CONNECTIONESS_CTE_DONE_EVENT: u8 = 0x1B;
/// Sent when the Set Connectionless CTE Transmit disable is complete.
pub const GAP_END_CONNECTIONESS_CTE_DONE_EVENT: u8 = 0x1C;
/// Sent when the periodic advertising sync transfer received.
pub const GAP_PERI_ADV_SYNC_TRAN_RECEIVED_EVENT: u8 = 0x1D;

#[doc = " GAP event header format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapEventHdr_t {
    #[doc = "!< GAP_MSG_EVENT and status"]
    pub hdr: tmos_event_hdr_t,
    #[doc = "!< GAP type of command. Ref: @ref GAP_MSG_EVENT_DEFINES"]
    pub opcode: u8,
}

#[doc = " Type of device."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapDevRec_t {
    #[doc = "!< Indicates advertising event type used by the advertiser: @ref GAP_ADVERTISEMENT_REPORT_TYPE_DEFINES"]
    pub eventType: u8,
    #[doc = "!< Address Type: @ref GAP_ADDR_TYPE_DEFINES"]
    pub addrType: u8,
    #[doc = "!< Device's Address"]
    pub addr: [u8; 6usize],
}

#[doc = " GAP_DEVICE_INIT_DONE_EVENT message format.  This message is sent to the\n app when the Device Initialization is done [initiated by calling\n GAP_DeviceInit()]."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapDeviceInitDoneEvent_t {
    #[doc = "!< GAP_MSG_EVENT and status"]
    pub hdr: tmos_event_hdr_t,
    #[doc = "!< GAP_DEVICE_INIT_DONE_EVENT"]
    pub opcode: u8,
    #[doc = "!< Device's BD_ADDR"]
    pub devAddr: [u8; 6usize],
    #[doc = "!< HC_LE_Data_Packet_Length"]
    pub dataPktLen: u16,
    #[doc = "!< HC_Total_Num_LE_Data_Packets"]
    pub numDataPkts: u8,
}
#[doc = " GAP_SIGNATURE_UPDATED_EVENT message format.  This message is sent to the\n app when the signature counter has changed.  This message is to inform the\n application in case it wants to save it to be restored on reboot or reconnect.\n This message is sent to update a connection's signature counter and to update\n this device's signature counter.  If devAddr == BD_ADDR, then this message pertains\n to this device."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapSignUpdateEvent_t {
    #[doc = "!< GAP_MSG_EVENT and status"]
    pub hdr: tmos_event_hdr_t,
    #[doc = "!< GAP_SIGNATURE_UPDATED_EVENT"]
    pub opcode: u8,
    #[doc = "!< Device's address type for devAddr"]
    pub addrType: u8,
    #[doc = "!< Device's BD_ADDR, could be own address"]
    pub devAddr: [u8; 6usize],
    #[doc = "!< new Signed Counter"]
    pub signCounter: u32,
}
#[doc = " GAP_DEVICE_INFO_EVENT message format.  This message is sent to the\n app during a Device Discovery Request, when a new advertisement or scan\n response is received."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapDeviceInfoEvent_t {
    #[doc = "!< GAP_MSG_EVENT and status"]
    pub hdr: tmos_event_hdr_t,
    #[doc = "!< GAP_DEVICE_INFO_EVENT"]
    pub opcode: u8,
    #[doc = "!< Advertisement Type: @ref GAP_ADVERTISEMENT_REPORT_TYPE_DEFINES"]
    pub eventType: u8,
    #[doc = "!< address type: @ref GAP_ADDR_TYPE_DEFINES"]
    pub addrType: u8,
    #[doc = "!< Address of the advertisement or SCAN_RSP"]
    pub addr: [u8; 6usize],
    #[doc = "!< Advertisement or SCAN_RSP RSSI"]
    pub rssi: i8,
    #[doc = "!< Length (in bytes) of the data field (evtData)"]
    pub dataLen: u8,
    #[doc = "!< Data field of advertisement or SCAN_RSP"]
    pub pEvtData: *mut u8,
}
#[doc = " GAP_DIRECT_DEVICE_INFO_EVENT message format.  This message is sent to the\n app during a Device Discovery Request, when a new advertisement or scan\n response is received."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapDirectDeviceInfoEvent_t {
    #[doc = "!< GAP_MSG_EVENT and status"]
    pub hdr: tmos_event_hdr_t,
    #[doc = "!< GAP_DIRECT_DEVICE_INFO_EVENT"]
    pub opcode: u8,
    #[doc = "!< Advertisement Type: @ref GAP_ADVERTISEMENT_REPORT_TYPE_DEFINES"]
    pub eventType: u8,
    #[doc = "!< address type: @ref GAP_ADDR_TYPE_DEFINES"]
    pub addrType: u8,
    #[doc = "!< Address of the advertisement or SCAN_RSP"]
    pub addr: [u8; 6usize],
    #[doc = "!< public or random address type"]
    pub directAddrType: u8,
    #[doc = "!< device address"]
    pub directAddr: [u8; 6usize],
    #[doc = "!< Advertisement or SCAN_RSP RSSI"]
    pub rssi: i8,
}
#[doc = " GAP_EXT_ADV_DEVICE_INFO_EVENT message format.  This message is sent to the\n app during a Device Discovery Request, when a new advertisement or scan\n response is received."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapExtAdvDeviceInfoEvent_t {
    #[doc = "!< GAP_MSG_EVENT and status"]
    pub hdr: tmos_event_hdr_t,
    #[doc = "!< GAP_EXT_ADV_DEVICE_INFO_EVENT"]
    pub opcode: u8,
    #[doc = "!< Advertisement Type: @ref GAP_ADVERTISEMENT_REPORT_TYPE_DEFINES"]
    pub eventType: u8,
    #[doc = "!< address type: @ref GAP_ADDR_TYPE_DEFINES"]
    pub addrType: u8,
    #[doc = "!< Address of the advertisement or SCAN_RSP"]
    pub addr: [u8; 6usize],
    #[doc = "!< Advertiser PHY on the primary advertising channel"]
    pub primaryPHY: u8,
    #[doc = "!< Advertiser PHY on the secondary advertising channel"]
    pub secondaryPHY: u8,
    #[doc = "!< Value of the Advertising SID subfield in the ADI field of the PDU"]
    pub advertisingSID: u8,
    #[doc = "!< Advertisement or SCAN_RSP power"]
    pub txPower: i8,
    #[doc = "!< Advertisement or SCAN_RSP RSSI"]
    pub rssi: i8,
    #[doc = "!< the interval of periodic advertising"]
    pub periodicAdvInterval: u16,
    #[doc = "!< public or random address type"]
    pub directAddressType: u8,
    #[doc = "!< device address"]
    pub directAddress: [u8; 6usize],
    #[doc = "!< Length (in bytes) of the data field (evtData)"]
    pub dataLen: u8,
    #[doc = "!< Data field of advertisement or SCAN_RSP"]
    pub pEvtData: *mut u8,
}
#[doc = " Type of device discovery (Scan) to perform."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapDevDiscReq_t {
    #[doc = "!< Requesting App's Task ID, used to return results"]
    pub taskID: u8,
    #[doc = "!< Discovery Mode: @ref GAP_DEVDISC_MODE_DEFINES"]
    pub mode: u8,
    #[doc = "!< TRUE for active scanning"]
    pub activeScan: u8,
    #[doc = "!< TRUE to only allow advertisements from devices in the white list."]
    pub whiteList: u8,
}
#[doc = " GAP_ADV_DATA_UPDATE_DONE_EVENT message format.  This message is sent to the\n app when Advertising Data Update is complete."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapAdvDataUpdateEvent_t {
    #[doc = "!< GAP_MSG_EVENT and status"]
    pub hdr: tmos_event_hdr_t,
    #[doc = "!< GAP_ADV_DATA_UPDATE_DONE_EVENT"]
    pub opcode: u8,
    #[doc = "!< TRUE if advertising data, FALSE if SCAN_RSP"]
    pub adType: u8,
}
#[doc = " GAP_PERIODIC_ADV_DEVICE_INFO_EVENT message format.  This message is sent to the\n app during Periodic Advertising Sync, when received a Periodic Advertising packet"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapPeriodicAdvDeviceInfoEvent_t {
    #[doc = "!< GAP_MSG_EVENT and status"]
    pub hdr: tmos_event_hdr_t,
    #[doc = "!< GAP_PERIODIC_ADV_DEVICE_INFO_EVENT"]
    pub opcode: u8,
    #[doc = "!< Identifying the periodic advertising train"]
    pub syncHandle: u16,
    #[doc = "!< Periodic advertising tx power,Units: dBm"]
    pub txPower: i8,
    #[doc = "!< Periodic advertising rssi,Units: dBm"]
    pub rssi: i8,
    pub unUsed: u8,
    #[doc = "!< Data complete"]
    pub dataStatus: u8,
    #[doc = "!< Length (in bytes) of the data field (evtData)"]
    pub dataLength: u8,
    #[doc = "!< Data field of periodic advertising data"]
    pub pEvtData: *mut u8,
}
#[doc = " GAP_DEVICE_DISCOVERY_EVENT message format. This message is sent to the\n Application after a scan is performed."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapDevDiscEvent_t {
    #[doc = "!< GAP_MSG_EVENT and status"]
    pub hdr: tmos_event_hdr_t,
    #[doc = "!< GAP_DEVICE_DISCOVERY_EVENT"]
    pub opcode: u8,
    #[doc = "!< Number of devices found during scan"]
    pub numDevs: u8,
    #[doc = "!< array of device records"]
    pub pDevList: *mut gapDevRec_t,
}

#[doc = " GAP_SYNC_ESTABLISHED_EVENT message format.  This message is sent to the\n app when the Periodic Advertising Sync Establish is complete."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapSyncEstablishedEvent_t {
    #[doc = "!< GAP_MSG_EVENT and status"]
    pub hdr: tmos_event_hdr_t,
    #[doc = "!< GAP_SYNC_ESTABLISHED_EVENT"]
    pub opcode: u8,
    #[doc = "!< Periodic advertising sync status"]
    pub status: u8,
    #[doc = "!< Identifying the periodic advertising train"]
    pub syncHandle: u16,
    #[doc = "!< Value of the Advertising SID subfield in the ADI field of the PDU"]
    pub advertisingSID: u8,
    #[doc = "!< Device address type: @ref GAP_ADDR_TYPE_DEFINES"]
    pub devAddrType: u8,
    #[doc = "!< Device address of sync"]
    pub devAddr: [u8; 6usize],
    #[doc = "!< Advertiser PHY"]
    pub advertisingPHY: u8,
    #[doc = "!< Periodic advertising interval"]
    pub periodicInterval: u16,
    #[doc = "!< Clock Accuracy"]
    pub clockAccuracy: u8,
}
#[doc = " GAP_SYNC_LOST_EVENT message format.  This message is sent to the\n app when the Periodic Advertising Sync timeout period."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapSyncLostEvent_t {
    #[doc = "!< GAP_MSG_EVENT and status"]
    pub hdr: tmos_event_hdr_t,
    #[doc = "!< GAP_SYNC_LOST_EVENT"]
    pub opcode: u8,
    #[doc = "!< Identifying the periodic advertising train"]
    pub syncHandle: u16,
}
#[doc = " GAP_SCAN_REQUEST_EVENT message format.  This message is sent to the\n app when the advertiser receives a SCAN_REQ PDU or an AUX_SCAN_REQ PDU"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapScanReqReseiveEvent_t {
    #[doc = "!< GAP_MSG_EVENT and status"]
    pub hdr: tmos_event_hdr_t,
    #[doc = "!< GAP_SCAN_REQUEST_EVENT"]
    pub opcode: u8,
    #[doc = "!< identifying the periodic advertising train"]
    pub advHandle: u8,
    #[doc = "!< the type of the address"]
    pub scannerAddrType: u8,
    #[doc = "!< the address of scanner device"]
    pub scannerAddr: [u8; 6usize],
}

#[doc = " GAP_LINK_ESTABLISHED_EVENT message format.  This message is sent to the app\n when the link request is complete.<BR>\n <BR>\n For an Observer, this message is sent to complete the Establish Link Request.<BR>\n For a Peripheral, this message is sent to indicate that a link has been created."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapEstLinkReqEvent_t {
    #[doc = "!< GAP_MSG_EVENT and status"]
    pub hdr: tmos_event_hdr_t,
    #[doc = "!< GAP_LINK_ESTABLISHED_EVENT"]
    pub opcode: u8,
    #[doc = "!< Device address type: @ref GAP_ADDR_TYPE_DEFINES"]
    pub devAddrType: u8,
    #[doc = "!< Device address of link"]
    pub devAddr: [u8; 6usize],
    #[doc = "!< Connection Handle from controller used to ref the device"]
    pub connectionHandle: u16,
    #[doc = "!< Connection formed as Central or Peripheral"]
    pub connRole: u8,
    #[doc = "!< Connection Interval"]
    pub connInterval: u16,
    #[doc = "!< Connection Latency"]
    pub connLatency: u16,
    #[doc = "!< Connection Timeout"]
    pub connTimeout: u16,
    #[doc = "!< Clock Accuracy"]
    pub clockAccuracy: u8,
}

#[doc = " GAP_LINK_PARAM_UPDATE_EVENT message format.  This message is sent to the app\n when the connection parameters update request is complete."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapLinkUpdateEvent_t {
    #[doc = "!< GAP_MSG_EVENT and status"]
    pub hdr: tmos_event_hdr_t,
    #[doc = "!< GAP_LINK_PARAM_UPDATE_EVENT"]
    pub opcode: u8,
    #[doc = "!< bStatus_t"]
    pub status: u8,
    #[doc = "!< Connection handle of the update"]
    pub connectionHandle: u16,
    #[doc = "!< Requested connection interval"]
    pub connInterval: u16,
    #[doc = "!< Requested connection latency"]
    pub connLatency: u16,
    #[doc = "!< Requested connection timeout"]
    pub connTimeout: u16,
}
#[doc = " GAP_LINK_TERMINATED_EVENT message format.  This message is sent to the\n app when a link to a device is terminated."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapTerminateLinkEvent_t {
    #[doc = "!< GAP_MSG_EVENT and status"]
    pub hdr: tmos_event_hdr_t,
    #[doc = "!< GAP_LINK_TERMINATED_EVENT"]
    pub opcode: u8,
    #[doc = "!< connection Handle"]
    pub connectionHandle: u16,
    #[doc = "!< termination reason from LL"]
    pub reason: u8,
    pub connRole: u8,
}

#[doc = " GAP_PHY_UPDATE_EVENT message format.  This message is sent to the app(GAP_MSG_EVENT)\n when the PHY update request is complete."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapPhyUpdateEvent_t {
    #[doc = "!< GAP_MSG_EVENT and status"]
    pub hdr: tmos_event_hdr_t,
    #[doc = "!< GAP_PHY_UPDATE_EVENT"]
    pub opcode: u8,
    #[doc = "!< bStatus_t"]
    pub status: u8,
    #[doc = "!< Connection handle of the update"]
    pub connectionHandle: u16,
    #[doc = "!< tx phy(GAP_PHY_VAL_TYPE)"]
    pub connTxPHYS: u8,
    #[doc = "!< rx phy(GAP_PHY_VAL_TYPE)"]
    pub connRxPHYS: u8,
}
#[doc = " gapRole Event Structure"]
#[repr(C)]
#[derive(Copy, Clone)]
pub union gapRoleEvent_t {
    #[doc = "!< GAP_MSG_EVENT and status."]
    pub gap: gapEventHdr_t,
    #[doc = "!< GAP initialization done."]
    pub initDone: gapDeviceInitDoneEvent_t,
    #[doc = "!< Discovery device information event structure."]
    pub deviceInfo: gapDeviceInfoEvent_t,
    #[doc = "!< Discovery direct device information event structure."]
    pub deviceDirectInfo: gapDirectDeviceInfoEvent_t,
    #[doc = "!< Advertising Data Update is complete."]
    pub dataUpdate: gapAdvDataUpdateEvent_t,
    #[doc = "!< Discovery periodic device information event structure."]
    pub devicePeriodicInfo: gapPeriodicAdvDeviceInfoEvent_t,
    #[doc = "!< Discovery extend advertising device information event structure."]
    pub deviceExtAdvInfo: gapExtAdvDeviceInfoEvent_t,
    #[doc = "!< Discovery complete event structure."]
    pub discCmpl: gapDevDiscEvent_t,
    #[doc = "!< sync established event structure."]
    pub syncEstEvt: gapSyncEstablishedEvent_t,
    #[doc = "!< sync lost event structure."]
    pub syncLostEvt: gapSyncLostEvent_t,
    #[doc = "!< Scan_Request_Received event structure."]
    pub scanReqEvt: gapScanReqReseiveEvent_t,
    #[doc = "!< Link complete event structure."]
    pub linkCmpl: gapEstLinkReqEvent_t,
    #[doc = "!< Link update event structure."]
    pub linkUpdate: gapLinkUpdateEvent_t,
    #[doc = "!< Link terminated event structure."]
    pub linkTerminate: gapTerminateLinkEvent_t,
    #[doc = "!< Link phy update event structure."]
    pub linkPhyUpdate: gapPhyUpdateEvent_t,
}

//

#[doc = " gapRole_States_t defined"]
pub type gapRole_States_t = ::core::ffi::c_ulong;

#[doc = " Observer Event Callback Function"]
pub type pfnGapObserverRoleEventCB_t = Option<unsafe extern "C" fn(pEvent: &gapRoleEvent_t)>;
#[doc = " Observer Callback Structure"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapRoleObserverCB_t {
    #[doc = "!< Event callback."]
    pub eventCB: pfnGapObserverRoleEventCB_t,
}

#[doc = " Passcode Callback Function"]
pub type pfnPasscodeCB_t =
    Option<unsafe extern "C" fn(deviceAddr: *mut u8, connectionHandle: u16, uiInputs: u8, uiOutputs: u8)>;

#[doc = " Pairing State Callback Function"]
pub type pfnPairStateCB_t = Option<unsafe extern "C" fn(connectionHandle: u16, state: u8, status: u8)>;

#[doc = " OOB Callback Function"]
pub type pfnOobCB_t =
    Option<unsafe extern "C" fn(deviceAddr: *mut u8, connectionHandle: u16, r_local: *mut u8, c_local: *mut u8)>;

#[doc = " Callback Registration Structure"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapBondCBs_t {
    #[doc = "!< Passcode callback"]
    pub passcodeCB:
        Option<unsafe extern "C" fn(deviceAddr: *mut u8, connectionHandle: u16, uiInputs: u8, uiOutputs: u8)>,
    #[doc = "!< Pairing state callback"]
    pub pairStateCB: Option<unsafe extern "C" fn(connectionHandle: u16, state: u8, status: u8)>,
    #[doc = "!< oob callback"]
    pub oobCB:
        Option<unsafe extern "C" fn(deviceAddr: *mut u8, connectionHandle: u16, r_local: *mut u8, c_local: *mut u8)>,
}

#[doc = " Callback structure - must be setup by the application and used when gapRoles_StartDevice() is called."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapRolesCBs_t {
    #[doc = "!< Whenever the device changes state"]
    pub pfnStateChange: Option<unsafe extern "C" fn(newState: gapRole_States_t, pEvent: *mut gapRoleEvent_t)>,
    #[doc = "!< When a valid RSSI is read from controller"]
    pub pfnRssiRead: Option<unsafe extern "C" fn(connHandle: u16, newRSSI: i8)>,
    #[doc = "!< When the connection parameteres are updated"]
    pub pfnParamUpdate:
        Option<unsafe extern "C" fn(connHandle: u16, connInterval: u16, connSlaveLatency: u16, connTimeout: u16)>,
}

#[doc = " Central Callback Structure"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapCentralRoleCB_t {
    #[doc = "!< RSSI callback. Callback when the device has read an new RSSI value during a connection."]
    pub rssiCB: Option<unsafe extern "C" fn(connHandle: u16, newRSSI: i8)>,
    #[doc = "!< Event callback. Central Event Callback Function"]
    pub eventCB: Option<unsafe extern "C" fn(pEvent: *mut gapRoleEvent_t)>,
    #[doc = "!< Length Change Event Callback. HCI Data Length Change Event Callback Function"]
    pub ChangCB: Option<unsafe extern "C" fn(connHandle: u16, maxTxOctets: u16, maxRxOctets: u16)>,
}

#[doc = " Type of device."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapScanRec_t {
    #[doc = "!< Indicates advertising event type used by the advertiser: @ref GAP_ADVERTISEMENT_REPORT_TYPE_DEFINES"]
    pub eventType: u8,
    #[doc = "!< Scan Address Type:0x00-Public Device Address or Public Identity Address 0x01-Random Device Address or Random (static) Identity Address"]
    pub addrType: u8,
    #[doc = "!< Device's Address"]
    pub addr: [u8; 6usize],
    pub rssi: i8,
}
#[doc = " Callback when the device has been started.  Callback event to\n the Notify of a state change."]
pub type gapRolesBroadcasterStateNotify_t = Option<unsafe extern "C" fn(newState: gapRole_States_t)>;
pub type gapRolesScanReqRecv_t = Option<unsafe extern "C" fn(pEvent: *mut gapScanRec_t)>;
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapRolesBroadcasterCBs_t {
    #[doc = "!< Whenever the device changes state"]
    pub pfnStateChange: gapRolesBroadcasterStateNotify_t,
    pub pfnScanRecv: gapRolesScanReqRecv_t,
}

/// GAP Role
pub struct GAPRole;

impl GAPRole {
    pub fn broadcaster_init() -> bStatus_t {
        unsafe { GAPRole_BroadcasterInit() }
    }

    pub fn observer_init() -> bStatus_t {
        unsafe { GAPRole_ObserverInit() }
    }

    pub fn peripheral_init() -> bStatus_t {
        unsafe { GAPRole_PeripheralInit() }
    }

    pub fn central_init() -> bStatus_t {
        unsafe { GAPRole_CentralInit() }
    }
}

// GAP
extern "C" {

    #[doc = " @brief   Initialization function for the GAP Role Task.\n\n @param   None.\n\n @return  SUCCESS,bleInvalidRange"]
    pub fn GAPRole_BroadcasterInit() -> bStatus_t;

    #[doc = " @brief   Does the Broadcaster receive scan request call initialization.\n\n @param   pAppCallbacks - pointer to application callbacks.\n\n @return  None"]
    pub fn GAPRole_BroadcasterSetCB(pAppCallbacks: &'static gapRolesBroadcasterCBs_t);

    #[doc = " @internal\n\n @brief   Observer Profile Task initialization function.\n\n @param   None.\n\n @return  SUCCESS,bleInvalidRange"]
    pub fn GAPRole_ObserverInit() -> bStatus_t;

    // use static lifetime, the holder struct must be static
    #[doc = " @brief   Start the device in Observer role.  This function is typically\n          called once during system startup.\n\n @param   pAppCallbacks - pointer to application callbacks\n\n @return  SUCCESS: Operation successful.<BR>\n          bleAlreadyInRequestedMode: Device already started.<BR>"]
    pub fn GAPRole_ObserverStartDevice(pAppCallbacks: &'static gapRoleObserverCB_t) -> bStatus_t;

    #[doc = " @brief   Start a device discovery scan.\n\n @param   mode - discovery mode: @ref GAP_DEVDISC_MODE_DEFINES\n @param   activeScan - TRUE to perform active scan\n @param   whiteList - TRUE to only scan for devices in the white list\n\n @return  SUCCESS: Discovery scan started.<BR>\n          bleIncorrectMode: Invalid profile role.<BR>\n          bleAlreadyInRequestedMode: Not available.<BR>"]
    pub fn GAPRole_ObserverStartDiscovery(mode: u8, activeScan: u8, whiteList: u8) -> bStatus_t;

    #[doc = " @brief   Cancel a device discovery scan.\n\n @return  SUCCESS: Cancel started.<BR>\n          bleInvalidTaskID: Not the task that started discovery.<BR>\n          bleIncorrectMode: Not in discovery mode.<BR>"]
    pub fn GAPRole_ObserverCancelDiscovery() -> bStatus_t;

    #[doc = " @internal\n\n @brief   Initialization function for the GAP Role Task.\n          This is called during initialization and should contain\n          any application specific initialization (ie. hardware\n          initialization/setup, table initialization, power up\n          notificaiton ... ).\n\n @param   None.\n\n @return  SUCCESS,bleInvalidRange"]
    pub fn GAPRole_PeripheralInit() -> bStatus_t;

    // Use static lifetime, the holder struct must be static
    #[doc = " @brief   Does the device initialization.  Only call this function once.\n\n @param   pAppCallbacks - pointer to application callbacks.\n\n @return  SUCCESS or bleAlreadyInRequestedMode"]
    pub fn GAPRole_PeripheralStartDevice(
        taskid: u8,
        pCB: &'static gapBondCBs_t,
        pAppCallbacks: &'static gapRolesCBs_t,
    ) -> bStatus_t;

    #[doc = " @brief   Update the parameters of an existing connection\n\n @param   connHandle - the connection Handle\n @param   connIntervalMin - minimum connection interval in 1.25ms units\n @param   connIntervalMax - maximum connection interval in 1.25ms units\n @param   latency - the new slave latency\n @param   connTimeout - the new timeout value\n @param   taskId - taskID will recv L2CAP_SIGNAL_EVENT message\n\n @return  SUCCESS, bleNotConnected or bleInvalidRange"]
    pub fn GAPRole_PeripheralConnParamUpdateReq(
        connHandle: u16,
        connIntervalMin: u16,
        connIntervalMax: u16,
        latency: u16,
        connTimeout: u16,
        taskId: u8,
    ) -> bStatus_t;

    #[doc = " @internal\n\n @brief   Central Profile Task initialization function.\n\n @param   None.\n\n @return  SUCCESS,bleInvalidRange"]
    pub fn GAPRole_CentralInit() -> bStatus_t;

    #[doc = " @brief   Start the device in Central role.  This function is typically\n          called once during system startup.\n\n @param   pAppCallbacks - pointer to application callbacks\n\n @return  SUCCESS: Operation successful.<BR>\n          bleAlreadyInRequestedMode: Device already started.<BR>"]
    pub fn GAPRole_CentralStartDevice(
        taskid: u8,
        pCB: *mut gapBondCBs_t,
        pAppCallbacks: *mut gapCentralRoleCB_t,
    ) -> bStatus_t;

    #[doc = " @brief   Set a GAP Role parameter.\n\n @note    You can call this function with a GAP Parameter ID and it will set a GAP Parameter.\n\n @param   param - Profile parameter ID: @ref GAPROLE_PROFILE_PARAMETERS\n @param   len - length of data to write\n @param   pValue - pointer to data to write.  This is dependent on the parameter ID and\n                   WILL be cast to the appropriate data type (example: data type of uint16_t\n                   will be cast to uint16_t pointer).\n\n @return  SUCCESS or INVALIDPARAMETER (invalid paramID)"]
    pub fn GAPRole_SetParameter(param: u16, len: u16, pValue: *const ::core::ffi::c_void) -> bStatus_t;

    #[doc = " @brief   Get a GAP Role parameter.\n\n @note    You can call this function with a GAP Parameter ID and it will get a GAP Parameter.\n\n @param   param - Profile parameter ID: @ref GAPROLE_PROFILE_PARAMETERS\n @param   pValue - pointer to location to get the value.  This is dependent on\n          the parameter ID and WILL be cast to the appropriate\n          data type (example: data type of uint16_t will be cast to\n          uint16_t pointer).\n\n @return      SUCCESS or INVALIDPARAMETER (invalid paramID)"]
    pub fn GAPRole_GetParameter(param: u16, pValue: *mut ::core::ffi::c_void) -> bStatus_t;

    #[doc = " @brief       Terminates the existing connection.\n\n @return      SUCCESS or bleIncorrectMode"]
    pub fn GAPRole_TerminateLink(connHandle: u16) -> bStatus_t;

    #[doc = " @brief   Read Rssi Cmd.\n\n @param   connHandle - connection handle\n\n @return  bStatus_t: HCI Error Code.<BR>\n"]
    pub fn GAPRole_ReadRssiCmd(connHandle: u16) -> bStatus_t;

    // Use static lifetime, the holder struct must be static
    #[doc = " @brief   Does the device initialization.  Only call this function once.\n\n @param   pAppCallbacks - pointer to application callbacks.\n\n @return  SUCCESS or bleAlreadyInRequestedMode"]
    pub fn GAPRole_BroadcasterStartDevice(pAppCallbacks: &'static gapRolesBroadcasterCBs_t) -> bStatus_t;

    #[doc = " @brief   used to cancel the HCI_LE_Periodic_Advertising_Create_Sync command while\n          it is pending.\n\n @param   None.\n\n @return  bStatus_t: HCI Error Code.<BR>\n"]
    pub fn GAPRole_CancelSync() -> bStatus_t;

}

// FUNCTIONS - Initialization and Configuration
extern "C" {
    #[doc = " @brief   Set a GAP Parameter value.  Use this function to change  the default GAP parameter values.\n\n @param   paramID - parameter ID: @ref GAP_PARAMETER_ID_DEFINES\n @param   paramValue - new param value\n\n @return  SUCCESS or INVALIDPARAMETER (invalid paramID)"]
    pub fn GAP_SetParamValue(paramID: u16, paramValue: u16) -> bStatus_t;
    #[doc = " @brief   Get a GAP Parameter value.\n\n @note    This function is the same as GAP_PasskeyUpdate(), except that\n          the passkey is passed in as a non-string format.\n\n @param   paramID - parameter ID: @ref GAP_PARAMETER_ID_DEFINES\n\n @return  GAP Parameter value or 0xFFFF if invalid"]
    pub fn GAP_GetParamValue(paramID: u16) -> u16;
    #[doc = " @brief   Setup the device's address type.  If ADDRTYPE_PRIVATE_RESOLVE is selected,\n          the address will change periodically.\n\n @param   addrType - @ref GAP_ADDR_TYPE_DEFINES\n @param   pStaticAddr - Only used with ADDRTYPE_STATIC or ADDRTYPE_PRIVATE_NONRESOLVE type\n                   NULL to auto generate otherwise the application can specify the address value\n\n @return  SUCCESS: address type updated,<BR>\n          bleNotReady: Can't be called until GAP_DeviceInit() is called\n                   and the init process is completed\n          bleIncorrectMode: can't change with an active connection,or INVALIDPARAMETER\n          If return value isn't SUCCESS, the address type remains the same as before this call."]
    pub fn GAP_ConfigDeviceAddr(addrType: u8, pStaticAddr: *mut u8) -> bStatus_t;
    #[doc = " @brief   Resolves a private address against an IRK.\n\n @param(in)   pIRK - pointer to the IRK\n @param(in)   pAddr - pointer to the Resolvable Private address\n\n @param(out)  pIRK\n @param(out)  pAddr\n\n @return  SUCCESS: match,<BR>\n          FAILURE: don't match,<BR>\n          INVALIDPARAMETER: parameters invalid<BR>"]
    pub fn GAP_ResolvePrivateAddr(pIRK: *mut u8, pAddr: *mut u8) -> bStatus_t;
    #[doc = " @brief   Setup or change advertising and scan response data.\n\n @note    if the return status from this function is SUCCESS,the task isn't complete\n          until the GAP_ADV_DATA_UPDATE_DONE_EVENT is sent to the calling application task.\n\n @param   taskID - task ID of the app requesting the change\n @param   adType - TRUE - advertisement data, FALSE  - scan response data\n @param   dataLen - Octet length of advertData\n @param   pAdvertData - advertising or scan response data\n\n @return  SUCCESS: data accepted\n          bleIncorrectMode: invalid profile role"]
    pub fn GAP_UpdateAdvertisingData(taskID: u8, adType: u8, dataLen: u16, pAdvertData: *mut u8) -> bStatus_t;
}
