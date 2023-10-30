#![allow(non_camel_case_types, non_snake_case, non_upper_case_globals)]

use core::num::NonZeroU8;

// pub type bStatus_t = u8;
// SUCCESS(0x00):指令按预期执行。
// INVALIDPARAMETER(0x02):无效的连接句柄或请求字段。
// MSG_BUFFER_NOT_AVAIL (0x04):HCI 缓冲区不可用。请稍后重试。
// bleNotConnected(0x14):设备未连接。
// blePending(0x17):
// 当返回到客户端功能时，服务器或 GATT 子过程正在进行中，有待处理的响应。
// 返回服务器功能时，来自客户端的确认待处理。
// bleTimeout(0x16):上一个事务超时。重新连接之前，无法发送 ATT 或 GATT 消息。
// bleMemAllocError(0x13):发生内存分配错误
// bleLinkEncrypted(0x19):链接已加密。不要在加密的链接上发送包含身份验证签
/*
#define FAILURE                         0x01   //!< Failure
#define INVALIDPARAMETER                0x02   //!< Invalid request field
#define INVALID_TASK                    0x03   //!< Task ID isn't setup properly
#define MSG_BUFFER_NOT_AVAIL            0x04   //!< No buffer is available.
#define INVALID_MSG_POINTER             0x05   //!< No message pointer.
#define INVALID_EVENT_ID                0x06   //!< Invalid event id.
#define INVALID_TIMEOUT                 0x07   //!< Invalid timeout.
#define NO_TIMER_AVAIL                  0x08   //!< No event is available.
#define NV_OPER_FAILED                  0x0A   //!< read a data item to NV failed.
#define INVALID_MEM_SIZE                0x0B   //!< The tokens take up too much space and don't fit into Advertisement data and Scan Response Data

#define bleInvalidTaskID                INVALID_TASK  //!< Task ID isn't setup properly
#define bleEecKeyRequestRejected        0x06   //!< key missing
#define bleNotReady                     0x10   //!< Not ready to perform task
#define bleAlreadyInRequestedMode       0x11   //!< Already performing that task
#define bleIncorrectMode                0x12   //!< Not setup properly to perform that task
#define bleMemAllocError                0x13   //!< Memory allocation error occurred
#define bleNotConnected                 0x14   //!< Can't perform function when not in a connection
#define bleNoResources                  0x15   //!< There are no resource available
#define blePending                      0x16   //!< Waiting
#define bleTimeout                      0x17   //!< Timed out performing function
#define bleInvalidRange                 0x18   //!< A parameter is out of range
#define bleLinkEncrypted                0x19   //!< The link is already encrypted
#define bleProcedureComplete            0x1A   //!< The Procedure is completed
#define bleInvalidMtuSize               0x1B   //!< SDU size is larger than peer MTU.
 */
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Status {
    Ok,
    Error(NonZeroU8),
}

impl Status {
    pub fn is_ok(self) -> bool {
        self == Status::Ok
    }

    pub fn is_err(self) -> bool {
        !self.is_ok()
    }
}

pub type bStatus_t = Status;

pub type tmosTaskID = u8;
pub type tmosEvents = u16;
pub type tmosTimer = u32;
pub type BOOL = u8;


/* Tx_POWER define(Accuracy:±2dBm) */
pub const LL_TX_POWEER_MINUS_16_DBM: u8 = 0x01;
pub const LL_TX_POWEER_MINUS_12_DBM: u8 = 0x02;
pub const LL_TX_POWEER_MINUS_8_DBM: u8 = 0x04;
pub const LL_TX_POWEER_MINUS_5_DBM: u8 = 0x07;
pub const LL_TX_POWEER_MINUS_3_DBM: u8 = 0x09;
pub const LL_TX_POWEER_MINUS_1_DBM: u8 = 0x0B;
pub const LL_TX_POWEER_0_DBM: u8 = 0x0D;
pub const LL_TX_POWEER_1_DBM: u8 = 0x0F;
pub const LL_TX_POWEER_2_DBM: u8 = 0x13;
pub const LL_TX_POWEER_3_DBM: u8 = 0x17;
pub const LL_TX_POWEER_4_DBM: u8 = 0x1D;
pub const LL_TX_POWEER_5_DBM: u8 = 0x29;
pub const LL_TX_POWEER_6_DBM: u8 = 0x3D;

/// BLE library config struct
/// Library initialization call BLE_LibInit function
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct bleConfig_t {
    /// library memory start address
    pub MEMAddr: u32,
    /// library memory size, > 4k
    pub MEMLen: u16,
    /// SNV flash start address( if NULL,bonding information will not be saved )
    pub SNVAddr: u32,
    /// SNV flash block size ( default 256 )
    pub SNVBlock: u16,
    /// SNV flash block number ( default 1 )
    pub SNVNum: u8,
    /// Maximum number of sent and received packages cached by the controller( default 5 )
    /// Must be greater than the number of connections.
    pub BufNumber: u8,
    /// Maximum length (in octets) of the data portion of each HCI data packet( default 27 )
    // SC enable,must be greater than 69
    // ATT_MTU = BufMaxLen-4,Range[23,ATT_MAX_MTU_SIZE]
    pub BufMaxLen: u16,
    /// Maximum number of TX data in a connection event ( default 1 )
    pub TxNumEvent: u8,
    /// Maximum number of RX data in a connection event ( default equal to BufNumber )
    pub RxNumEvent: u8,
    /// Transmit power level( default LL_TX_POWEER_0_DBM(0dBm) )
    pub TxPower: u8,
    /// Wake up time value in one system count
    pub WakeUpTime: u8,
    /// system clock select
    /// bit0-1 00: LSE(32768Hz) 01:LSI(32000Hz) 10:LSI(32768Hz)
    /// bit7:  1: ble timer(HSE)(must disable sleep)
    pub SelRTCClock: u8,
    /// Connect number,lower two bits are peripheral number,followed by central number
    pub ConnectNumber: u8,
    /// Wait rf start window(us)
    pub WindowWidening: u8,
    /// Wait event arrive window in one system clock
    pub WaitWindow: u8,
    /// MAC address, little-endian
    pub MacAddr: [u8; 6usize],
    /// Register a program that generate a random seed
    /// SYS_GetSysTickCnt
    pub srandCB: Option<unsafe extern "C" fn() -> u32>,
    /// Register a program that set idle
    pub sleepCB: Option<unsafe extern "C" fn(arg1: u32) -> u32>,
    /// Register a program that read the current temperature,determine whether calibration is need
    /// HAL_GetInterTempValue
    pub tsCB: Option<unsafe extern "C" fn() -> u16>,
    /// Register a program that LSI clock calibration
    /// Lib_Calibration_LSI
    pub rcCB: Option<unsafe extern "C" fn()>,
    /// Register a program that library status callback
    pub staCB: Option<unsafe extern "C" fn(code: u8, status: u32)>,
    /// Register a program that read flash
    pub readFlashCB: Option<unsafe extern "C" fn(addr: u32, num: u32, pBuf: *mut u32) -> u32>,
    /// Register a program that write flash
    pub writeFlashCB: Option<unsafe extern "C" fn(addr: u32, num: u32, pBuf: *mut u32) -> u32>,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct bleClockConfig_t {
    pub getClockValue: Option<unsafe extern "C" fn() -> u32>,
    /// The maximum count value
    pub ClockMaxCount: u32,
    /// The timing clock frequency(Hz)
    pub ClockFrequency: u16,
    /// The timing clock accuracy(ppm)
    pub ClockAccuracy: u16,
    pub irqEnable: u8,
}

pub const LLE_MODE_BASIC: u8 = 0;
pub const LLE_MODE_AUTO: u8 = 1;

pub const LLE_WHITENING_ON: u8 = 0;
pub const LLE_WHITENING_OFF: u8 = 2;

pub const LLE_MODE_PHY_MODE_MASK: u8 = 48;
pub const LLE_MODE_PHY_1M: u8 = 0;
pub const LLE_MODE_PHY_2M: u8 = 16;
pub const LLE_MODE_PHY_CODED_S8: u8 = 32;
pub const LLE_MODE_PHY_CODED_S2: u8 = 48;
pub const LLE_MODE_EX_CHANNEL: u8 = 64;

pub const LLE_MODE_NON_RSSI: u8 = 128;

// `sta` in rfStatusCB
// RF_Tx
pub const TX_MODE_TX_FINISH: u32 = 1;
pub const TX_MODE_TX_FAIL: u32 = 17;
pub const TX_MODE_TX_TIMEOUT: u32 = 17;
/// auto tx mode receive data(ack) and enter idle state
pub const TX_MODE_RX_DATA: u32 = 2;
pub const TX_MODE_RX_TIMEOUT: u32 = 18;
pub const TX_MODE_HOP_SHUT: u32 = 34;

// RF_Rx
/// basic or auto rx mode receive data
pub const RX_MODE_RX_DATA: u32 = 3;
/// auto rx mode sends data(ack) successfully and enters idle state
pub const RX_MODE_TX_FINISH: u32 = 4;
/// auto rx mode fail to send data and enter idle state
pub const RX_MODE_TX_FAIL: u32 = 20;
pub const RX_MODE_TX_TIMEOUT: u32 = 20;
pub const RX_MODE_HOP_SHUT: u32 = 36;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct rfConfig_t {
    #[doc = "!< BIT0   0=basic, 1=auto def@LLE_MODE_TYPE\n!< BIT1   0=whitening on, 1=whitening off def@LLE_WHITENING_TYPE\n!< BIT4-5 00-1M  01-2M  10-coded(S8) 11-coded(S2) def@LLE_PHY_TYPE\n!< BIT6   0=data channel(0-39)\n!<        1=rf frequency (2400000kHz-2483500kHz)\n!< BIT7   0=the first byte of the receive buffer is rssi\n!<        1=the first byte of the receive buffer is package type"]
    pub LLEMode: u8,
    #[doc = "!< rf channel(0-39)"]
    pub Channel: u8,
    #[doc = "!< rf frequency (2400000kHz-2483500kHz)"]
    pub Frequency: u32,
    #[doc = "!< access address,32bit PHY address"]
    pub accessAddress: u32,
    #[doc = "!< crc initial value"]
    pub CRCInit: u32,
    #[doc = "!< status call back"]
    pub rfStatusCB: Option<unsafe extern "C" fn(sta: u8, rsr: u8, rxBuf: *mut u8)>,
    #[doc = "!< indicating  Used and Unused data channels.Every channel is represented with a\n!< bit positioned as per the data channel index,The LSB represents data channel index 0"]
    pub ChannelMap: u32,
    pub _Resv: u8,
    #[doc = "!< The heart package interval shall be an integer multiple of 100ms"]
    pub HeartPeriod: u8,
    #[doc = "!< hop period( T=32n*RTC clock ),default is 8"]
    pub HopPeriod: u8,
    #[doc = "!< indicate the hopIncrement used in the data channel selection algorithm,default is 17"]
    pub HopIndex: u8,
    #[doc = "!< Maximum data length received in rf-mode(default 251)"]
    pub RxMaxlen: u8,
    #[doc = "!< Maximum data length transmit in rf-mode(default 251)"]
    pub TxMaxlen: u8,
}

#[link(name = "CH58xBLE")]
extern "C" {
    /// "CH58x_BLE_LIB_V1.9"
    pub static VER_LIB: [core::ffi::c_char; 0];

    #[doc = " @brief   Init BLE lib. RTC will be occupied at the same time.\n\n @param   pCfg - config of BLE lib\n\n @return  0-success. error defined @ ERR_LIB_INIT"]
    pub fn BLE_LibInit(pCfg: *const bleConfig_t) -> bStatus_t;

    #[doc = " @brief   ble register reset and rf calibration\n\n @param   None\n\n @return  None"]
    pub fn BLE_RegInit();
}

extern "C" {

    #[doc = " @brief   RF_PHY Profile Task initialization function.\n\n @param   None.\n\n @return  0 - success."]
    pub fn RF_RoleInit() -> bStatus_t;

    #[doc = " @brief   rf config.\n\n @param   pConfig - rf config parameters\n\n @return  0 - success."]
    pub fn RF_Config(pConfig: *mut rfConfig_t) -> bStatus_t;

    #[doc = " @brief   rx mode.\n\n @param   txBuf - rx mode tx data\n @param   txLen - rx mode tx length(0-251)\n @param   pktRxType - rx mode rx package type\n                      broadcast type(0xFF):receive all matching types,\n                      others:receive match type or broadcast type\n @param   pktTxType - rx mode tx package type(auto mode)\n                      broadcast type(0xFF):received by all matching types;\n                      others:only received by matching type\n\n @return  0 - success. 1-access address error 2-busy"]
    pub fn RF_Rx(txBuf: *mut u8, txLen: u8, pktRxType: u8, pktTxType: u8) -> bStatus_t;

    #[doc = " @brief   tx mode.\n\n @param   txBuf - tx mode tx data\n @param   txLen - tx mode tx length(0-251)\n @param   pktTxType - tx mode tx package type\n                      broadcast type(0xFF):received by all matching types;\n                      others:only received by matching type\n @param   pktRxType - tx mode rx package type(auto mode)\n                      broadcast type(0xFF):receive all matching types,\n                      others:receive match type or broadcast type\n\n @return  0 - success. 1-access address error 2-busy"]
    pub fn RF_Tx(txBuf: *mut u8, txLen: u8, pktTxType: u8, pktRxType: u8) -> bStatus_t;

    #[doc = " @brief   shut down,stop tx/rx mode.\n\n @param   None.\n\n @return  0 - success."]
    pub fn RF_Shut() -> bStatus_t;

}

// TMOS
extern "C" {

    // tmos
    // tmosTaskID, u8
    // tmosEvents, u16

    pub fn TMOS_ProcessEventRegister(
        eventCb: Option<unsafe extern "C" fn(taskID: tmosTaskID, event: tmosEvents) -> tmosEvents>,
    ) -> tmosTaskID;

    #[doc = " @brief   tmos system timer initialization\n\n @note    must initialization before call tmos task\n\n @param   fnGetClock - 0:system clock select RTC timer\n                   valid:system clock select extend input\n\n @return  SUCCESS if successful, FAILURE if failed."]
    pub fn TMOS_TimerInit(pClockConfig: *const bleClockConfig_t) -> bStatus_t;

    #[doc = " @brief   start a event after period of time\n\n @param   taskID - task ID to set event for\n @param   event - event to be notified with\n @param   time - timeout value\n\n @return  TRUE,FALSE."]
    pub fn tmos_start_task(taskID: tmosTaskID, event: tmosEvents, time: tmosTimer) -> BOOL;

    #[doc = " @brief   Process system\n\n @param   None.\n\n @return  None."]
    pub fn TMOS_SystemProcess();

    #[doc = " @brief   Get current system clock\n\n @param   None.\n\n @return  current system clock (in 0.625ms)"]
    pub fn TMOS_GetSystemClock() -> u32;
}

#[doc = " gapRole_States_t defined"]
pub type gapRole_States_t = ::core::ffi::c_ulong;

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

// GAPRole_SetParameter() parameters
pub const GAPROLE_PROFILEROLE: u16 = 768;
pub const GAPROLE_IRK: u16 = 769;
pub const GAPROLE_SRK: u16 = 770;
pub const GAPROLE_SIGNCOUNTER: u16 = 771;
pub const GAPROLE_BD_ADDR: u16 = 772;
pub const GAPROLE_ADVERT_ENABLED: u16 = 773;
pub const GAPROLE_ADVERT_DATA: u16 = 774;
pub const GAPROLE_SCAN_RSP_DATA: u16 = 775;
pub const GAPROLE_ADV_EVENT_TYPE: u16 = 776;
pub const GAPROLE_ADV_DIRECT_TYPE: u16 = 777;
pub const GAPROLE_ADV_DIRECT_ADDR: u16 = 778;
pub const GAPROLE_ADV_CHANNEL_MAP: u16 = 779;
pub const GAPROLE_ADV_FILTER_POLICY: u16 = 780;
pub const GAPROLE_STATE: u16 = 781;
pub const GAPROLE_MAX_SCAN_RES: u16 = 782;
pub const GAPROLE_MIN_CONN_INTERVAL: u16 = 785;
pub const GAPROLE_MAX_CONN_INTERVAL: u16 = 786;
pub const GAPROLE_PHY_TX_SUPPORTED: u16 = 787;
pub const GAPROLE_PHY_RX_SUPPORTED: u16 = 788;
pub const GAPROLE_PERIODIC_ADVERT_DATA: u16 = 789;
pub const GAPROLE_PERIODIC_ADVERT_ENABLED: u16 = 790;
pub const GAPROLE_CTE_CONNECTIONLESS_ENABLED: u16 = 791;

pub const TGAP_GEN_DISC_ADV_MIN: u16 = 0;
pub const TGAP_LIM_ADV_TIMEOUT: u16 = 1;
pub const TGAP_DISC_SCAN: u16 = 2;
pub const TGAP_DISC_ADV_INT_MIN: u16 = 3;
pub const TGAP_DISC_ADV_INT_MAX: u16 = 4;
pub const TGAP_DISC_SCAN_INT: u16 = 5;
pub const TGAP_DISC_SCAN_WIND: u16 = 6;
pub const TGAP_CONN_EST_INT_MIN: u16 = 7;
pub const TGAP_CONN_EST_INT_MAX: u16 = 8;
pub const TGAP_CONN_EST_SCAN_INT: u16 = 9;
pub const TGAP_CONN_EST_SCAN_WIND: u16 = 10;
pub const TGAP_CONN_EST_HIGH_SCAN_INT: u16 = 11;
pub const TGAP_CONN_EST_HIGH_SCAN_WIND: u16 = 12;
pub const TGAP_CONN_EST_SUPERV_TIMEOUT: u16 = 13;
pub const TGAP_CONN_EST_LATENCY: u16 = 14;
pub const TGAP_CONN_EST_MIN_CE_LEN: u16 = 15;
pub const TGAP_CONN_EST_MAX_CE_LEN: u16 = 16;
pub const TGAP_PRIVATE_ADDR_INT: u16 = 17;
pub const TGAP_SM_TIMEOUT: u16 = 18;
pub const TGAP_SM_MIN_KEY_LEN: u16 = 19;
pub const TGAP_SM_MAX_KEY_LEN: u16 = 20;
pub const TGAP_FILTER_ADV_REPORTS: u16 = 21;
pub const TGAP_SCAN_RSSI_MIN: u16 = 22;
pub const TGAP_REJECT_CONN_PARAMS: u16 = 23;
pub const TGAP_AUTH_TASK_ID: u16 = 24;
pub const TGAP_ADV_TX_POWER: u16 = 25;
pub const TGAP_ADV_PRIMARY_PHY: u16 = 26;
pub const TGAP_ADV_SECONDARY_PHY: u16 = 27;
pub const TGAP_ADV_SECONDARY_MAX_SKIP: u16 = 28;
pub const TGAP_ADV_ADVERTISING_SID: u16 = 29;
pub const TGAP_ADV_SCAN_REQ_NOTIFY: u16 = 30;
pub const TGAP_ADV_ADVERTISING_DURATION: u16 = 31;
pub const TGAP_ADV_MAX_EVENTS: u16 = 32;
pub const TGAP_DISC_SCAN_PHY: u16 = 33;
pub const TGAP_DISC_SCAN_CODED_INT: u16 = 34;
pub const TGAP_DISC_SCAN_CODED_WIND: u16 = 35;
pub const TGAP_DISC_SCAN_DURATION: u16 = 36;
pub const TGAP_DISC_SCAN_PERIOD: u16 = 37;
pub const TGAP_CONN_EST_INT_PHY: u16 = 38;
pub const TGAP_CONN_EST_2M_INT_MIN: u16 = 39;
pub const TGAP_CONN_EST_2M_INT_MAX: u16 = 40;
pub const TGAP_CONN_EST_2M_SUPERV_TIMEOUT: u16 = 41;
pub const TGAP_CONN_EST_2M_LATENCY: u16 = 42;
pub const TGAP_CONN_EST_2M_MIN_CE_LEN: u16 = 43;
pub const TGAP_CONN_EST_2M_MAX_CE_LEN: u16 = 44;
pub const TGAP_CONN_EST_CODED_INT_MIN: u16 = 45;
pub const TGAP_CONN_EST_CODED_INT_MAX: u16 = 46;
pub const TGAP_CONN_EST_CODED_SCAN_INT: u16 = 47;
pub const TGAP_CONN_EST_CODED_SCAN_WIND: u16 = 48;
pub const TGAP_CONN_EST_CODED_HIGH_SCAN_INT: u16 = 49;
pub const TGAP_CONN_EST_CODED_HIGH_SCAN_WIND: u16 = 50;
pub const TGAP_CONN_EST_CODED_SUPERV_TIMEOUT: u16 = 51;
pub const TGAP_CONN_EST_CODED_LATENCY: u16 = 52;
pub const TGAP_CONN_EST_CODED_MIN_CE_LEN: u16 = 53;
pub const TGAP_CONN_EST_CODED_MAX_CE_LEN: u16 = 54;
pub const TGAP_PERIODIC_ADV_INT_MIN: u16 = 55;
pub const TGAP_PERIODIC_ADV_INT_MAX: u16 = 56;
pub const TGAP_PERIODIC_ADV_PROPERTIES: u16 = 57;
pub const TGAP_SCAN_MAX_LENGTH: u16 = 58;
pub const TGAP_AFH_CHANNEL_MDOE: u16 = 59;
pub const TGAP_CTE_TYPE: u16 = 60;
pub const TGAP_CTE_LENGTH: u16 = 61;
pub const TGAP_CTE_COUNT: u16 = 62;
pub const TGAP_LENGTH_OF_SWITCHING_PATTERN: u16 = 63;
pub const TGAP_ADV_PRIMARY_PHY_OPTIONS: u16 = 64;
pub const TGAP_ADV_SECONDARY_PHY_OPTIONS: u16 = 65;
pub const TGAP_PARAMID_MAX: u16 = 66;

// GAPROLE_SCAN_RSP_DATA
pub const GAP_ADTYPE_FLAGS: u8 = 1;
pub const GAP_ADTYPE_16BIT_MORE: u8 = 2;
pub const GAP_ADTYPE_16BIT_COMPLETE: u8 = 3;
pub const GAP_ADTYPE_32BIT_MORE: u8 = 4;
pub const GAP_ADTYPE_32BIT_COMPLETE: u8 = 5;
pub const GAP_ADTYPE_128BIT_MORE: u8 = 6;
pub const GAP_ADTYPE_128BIT_COMPLETE: u8 = 7;
/// Shortened local name
pub const GAP_ADTYPE_LOCAL_NAME_SHORT: u8 = 8;
pub const GAP_ADTYPE_LOCAL_NAME_COMPLETE: u8 = 9;
/// TX Power Level: -127 to +127 dBm
pub const GAP_ADTYPE_POWER_LEVEL: u8 = 10;
pub const GAP_ADTYPE_OOB_CLASS_OF_DEVICE: u8 = 13;
pub const GAP_ADTYPE_OOB_SIMPLE_PAIRING_HASHC: u8 = 14;
pub const GAP_ADTYPE_OOB_SIMPLE_PAIRING_RANDR: u8 = 15;
pub const GAP_ADTYPE_SM_TK: u8 = 16;
pub const GAP_ADTYPE_SM_OOB_FLAG: u8 = 17;
pub const GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE: u8 = 18;
pub const GAP_ADTYPE_SIGNED_DATA: u8 = 19;
pub const GAP_ADTYPE_SERVICES_LIST_16BIT: u8 = 20;
pub const GAP_ADTYPE_SERVICES_LIST_128BIT: u8 = 21;
pub const GAP_ADTYPE_SERVICE_DATA: u8 = 22;
pub const GAP_ADTYPE_PUBLIC_TARGET_ADDR: u8 = 23;
pub const GAP_ADTYPE_RANDOM_TARGET_ADDR: u8 = 24;
pub const GAP_ADTYPE_APPEARANCE: u8 = 25;
pub const GAP_ADTYPE_ADV_INTERVAL: u8 = 26;
pub const GAP_ADTYPE_LE_BD_ADDR: u8 = 27;
pub const GAP_ADTYPE_LE_ROLE: u8 = 28;
pub const GAP_ADTYPE_SIMPLE_PAIRING_HASHC_256: u8 = 29;
pub const GAP_ADTYPE_SIMPLE_PAIRING_RANDR_256: u8 = 30;
pub const GAP_ADTYPE_SERVICE_DATA_32BIT: u8 = 32;
pub const GAP_ADTYPE_SERVICE_DATA_128BIT: u8 = 33;
pub const GAP_ADTYPE_LE_SC_CONFIRMATION_VALUE: u8 = 34;
pub const GAP_ADTYPE_LE_SC_RANDOM_VALUE: u8 = 35;
pub const GAP_ADTYPE_URI: u8 = 36;
pub const GAP_ADTYPE_INDOOR_POSITION: u8 = 37;
pub const GAP_ADTYPE_TRAN_DISCOVERY_DATA: u8 = 38;
pub const GAP_ADTYPE_SUPPORTED_FEATURES: u8 = 39;
pub const GAP_ADTYPE_CHANNEL_MAP_UPDATE: u8 = 40;
pub const GAP_ADTYPE_PB_ADV: u8 = 41;
pub const GAP_ADTYPE_MESH_MESSAGE: u8 = 42;
pub const GAP_ADTYPE_MESH_BEACON: u8 = 43;
pub const GAP_ADTYPE_BIG_INFO: u8 = 44;
pub const GAP_ADTYPE_BROADCAST_CODE: u8 = 45;
pub const GAP_ADTYPE_RSL_SET_IDENT: u8 = 46;
pub const GAP_ADTYPE_ADV_INTERVAL_LONG: u8 = 47;
pub const GAP_ADTYPE_BROADCAST_NAME: u8 = 48;
pub const GAP_ADTYPE_ENCRYPTED_ADV_DATA: u8 = 49;
pub const GAP_ADTYPE_PERI_ADV_RSP_TIMING_INFO: u8 = 50;
pub const GAP_ADTYPE_ELECTRONIC_SHELF_LABEL: u8 = 52;
pub const GAP_ADTYPE_3D_INFO_DATA: u8 = 61;
/// Manufacturer Specific Data: first 2 octets contain the Company Identifier Code followed by the additional manufacturer specific data.
pub const GAP_ADTYPE_MANUFACTURER_SPECIFIC: u8 = 255;
pub const GAP_ADTYPE_FLAGS_LIMITED: u8 = 1;
pub const GAP_ADTYPE_FLAGS_GENERAL: u8 = 2;
pub const GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED: u8 = 4;


// gapRole_States_t
pub const GAPROLE_STATE_ADV_MASK: u32 = 15;
pub const GAPROLE_STATE_ADV_SHIFT: u32 = 0;
pub const GAPROLE_INIT: u32 = 0;
pub const GAPROLE_STARTED: u32 = 1;
pub const GAPROLE_ADVERTISING: u32 = 2;
pub const GAPROLE_WAITING: u32 = 3;
pub const GAPROLE_CONNECTED: u32 = 4;
pub const GAPROLE_CONNECTED_ADV: u32 = 5;
pub const GAPROLE_ERROR: u32 = 6;
pub const GAPROLE_STATE_PERIODIC_MASK: u32 = 240;
pub const GAPROLE_STATE_PERIODIC_SHIFT: u32 = 4;
pub const GAPROLE_PERIODIC_INVALID: u32 = 0;
pub const GAPROLE_PERIODIC_ENABLE: u32 = 16;
pub const GAPROLE_PERIODIC_WAIT: u32 = 32;
pub const GAPROLE_PERIODIC_ERROR: u32 = 48;
pub const GAPROLE_STATE_CTE_MASK: u32 = 3840;
pub const GAPROLE_STATE_CTE_SHIFT: u32 = 8;
pub const GAPROLE_CONNECTIONLESS_CTE_INVALID: u32 = 0;
pub const GAPROLE_CONNECTIONLESS_CTE_ENABLE: u32 = 256;
pub const GAPROLE_CONNECTIONLESS_CTE_WAIT: u32 = 512;
pub const GAPROLE_CONNECTIONLESS_CTE_ERROR: u32 = 768;
pub const GAPROLE_PERIODIC_STATE_VALID: u32 = 16777216;
pub const GAPROLE_CTE_T_STATE_VALID: u32 = 33554432;

// GAP
extern "C" {

    #[doc = " @brief   Initialization function for the GAP Role Task.\n\n @param   None.\n\n @return  SUCCESS,bleInvalidRange"]
    pub fn GAPRole_BroadcasterInit() -> bStatus_t;

    #[doc = " @brief   Set a GAP Role parameter.\n\n @note    You can call this function with a GAP Parameter ID and it will set a GAP Parameter.\n\n @param   param - Profile parameter ID: @ref GAPROLE_PROFILE_PARAMETERS\n @param   len - length of data to write\n @param   pValue - pointer to data to write.  This is dependent on the parameter ID and\n                   WILL be cast to the appropriate data type (example: data type of uint16_t\n                   will be cast to uint16_t pointer).\n\n @return  SUCCESS or INVALIDPARAMETER (invalid paramID)"]
    pub fn GAPRole_SetParameter(param: u16, len: u16, pValue: *const ::core::ffi::c_void) -> bStatus_t;

    #[doc = " @brief   Does the device initialization.  Only call this function once.\n\n @param   pAppCallbacks - pointer to application callbacks.\n\n @return  SUCCESS or bleAlreadyInRequestedMode"]
    pub fn GAPRole_BroadcasterStartDevice(pAppCallbacks: *mut gapRolesBroadcasterCBs_t) -> bStatus_t;

    #[doc = " @brief   Set a GAP Parameter value.  Use this function to change  the default GAP parameter values.\n\n @param   paramID - parameter ID: @ref GAP_PARAMETER_ID_DEFINES\n @param   paramValue - new param value\n\n @return  SUCCESS or INVALIDPARAMETER (invalid paramID)"]
    pub fn GAP_SetParamValue(paramID: u16, paramValue: u16) -> bStatus_t;

}
