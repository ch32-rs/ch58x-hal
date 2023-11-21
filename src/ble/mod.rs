use core::ffi::CStr;
use core::mem::MaybeUninit;
use core::num::NonZeroU8;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubBehavior, PubSubChannel, Subscriber};

use self::ffi::{tmos_event_hdr_t, tmos_msg_deallocate, tmos_msg_receive, BLE_PAControlInit, SYS_EVENT_MSG};
use crate::ble::ffi::{blePaControlConfig_t, tmos_start_task, BLE_RegInit};
use crate::gpio::sealed::Pin;
use crate::gpio::{AnyPin, Level, Output, OutputDrive};
use crate::{pac, println};

pub mod ffi;
pub mod gap;
pub mod gatt;
pub mod gatt_uuid;
pub mod gattservapp;

const HEAP_SIZE: usize = 1024 * 6;

#[repr(C, align(4))]
struct BLEHeap([MaybeUninit<u8>; HEAP_SIZE]);
static mut BLE_HEAP: BLEHeap = BLEHeap([MaybeUninit::uninit(); HEAP_SIZE]);

static CHANNEL: PubSubChannel<CriticalSectionRawMutex, TmosEvent, 4, 2, 2> = PubSubChannel::new();

pub type EventSubscriber = Subscriber<'static, CriticalSectionRawMutex, TmosEvent, 4, 2, 2>;

static mut PA_CONFIG: blePaControlConfig_t = blePaControlConfig_t {
    txEnableGPIO: 0,
    txDisableGPIO: 0,
    tx_pin: 0,
    rxEnableGPIO: 0,
    rxDisableGPIO: 0,
    rx_pin: 0,
};

#[derive(Clone, Debug)]
pub struct TmosEvent(pub *mut tmos_event_hdr_t);

unsafe impl Send for TmosEvent {}
unsafe impl Sync for TmosEvent {}

impl TmosEvent {
    /// Incoming GATT message
    pub const GATT_MSG_EVENT: u8 = 0xB0;
    /// Incoming GATT ServApp message
    pub const GATT_SERV_MSG_EVENT: u8 = 0xB1;
    /// Incoming GAP message
    pub const GAP_MSG_EVENT: u8 = 0xD0;

    #[inline(always)]
    pub fn message_id(&self) -> u8 {
        unsafe { (*self.0).event }
    }
}
impl Drop for TmosEvent {
    fn drop(&mut self) {
        // println!("TmosEvent::drop event={} status={}", (*self.0).event, (*self.0).status);
        unsafe {
            let _ = tmos_msg_deallocate(self.0 as *mut u8);
        }
    }
}

pub struct PaConfig {
    pub pa_tx_pin: AnyPin,
    pub pa_rx_pin: AnyPin,
    // TODO: handle pin polarity
    pub pa_tx_active_high: bool,
    pub pa_rx_active_high: bool,
}

fn configure_pa(pa_config: PaConfig) {
    let tx_pin = pa_config.pa_tx_pin;
    let rx_pin = pa_config.pa_rx_pin;

    unsafe {
        PA_CONFIG.tx_pin = tx_pin._pin() as u32;
        PA_CONFIG.txEnableGPIO = tx_pin.block().out.as_ptr() as u32;
        PA_CONFIG.txDisableGPIO = tx_pin.block().clr.as_ptr() as u32;

        PA_CONFIG.rx_pin = rx_pin._pin() as u32;
        PA_CONFIG.rxEnableGPIO = rx_pin.block().out.as_ptr() as u32;
        PA_CONFIG.rxDisableGPIO = rx_pin.block().clr.as_ptr() as u32;

        let tx_pin = Output::new(tx_pin, Level::High, OutputDrive::_5mA);
        let rx_pin = Output::new(rx_pin, Level::Low, OutputDrive::_5mA);

        core::mem::forget(tx_pin);
        core::mem::forget(rx_pin);

        println!("pa => {:x?}", PA_CONFIG);
    }
}

pub struct Config {
    pub mac_addr: MacAddress,
    pub pa_config: Option<PaConfig>,
}

impl Config {
    pub fn use_factory_mac_address(&mut self) -> &mut Self {
        self.mac_addr = MacAddress::from_raw(crate::isp::get_mac_address());
        self
    }

    pub fn enbale_pa(&mut self, tx_pin: AnyPin, rx_pin: AnyPin) -> &mut Self {
        self.pa_config = Some(PaConfig {
            pa_tx_pin: tx_pin,
            pa_rx_pin: rx_pin,
            pa_tx_active_high: true,
            pa_rx_active_high: true,
        });

        self
    }
}

impl Default for Config {
    fn default() -> Self {
        let mac_addr = MacAddress::from_msb([0x11, 0x22, 0x33, 0x44, 0x55, 0x66]);
        Config {
            mac_addr,
            pa_config: None,
        }
    }
}

const HAL_REG_INIT_EVENT: u16 = 0x2000;
const HAL_PA_INIT_EVENT: u16 = 0x1000;

// 120s in 625us unit(ticks)
const HAL_TMOS_TASK_INTERVAL: u32 = (120000) * 1000 / 625;

unsafe extern "C" fn hal_tmos_task(task_id: u8, events: u16) -> u16 {
    if events & SYS_EVENT_MSG != 0 {
        let msg = tmos_msg_receive(task_id);
        if !msg.is_null() {
            let event = TmosEvent(msg as _);
            CHANNEL.publish_immediate(event);
            // Dealloc in Drop
            // let _ = tmos_msg_deallocate(msg);
        }
        return events ^ SYS_EVENT_MSG;
    } else if events & HAL_REG_INIT_EVENT != 0 {
        BLE_RegInit();

        tmos_start_task(task_id, HAL_REG_INIT_EVENT, HAL_TMOS_TASK_INTERVAL);
        return events ^ HAL_REG_INIT_EVENT;
    } else if events & HAL_PA_INIT_EVENT != 0 {
        BLE_PAControlInit(&PA_CONFIG);

        return events ^ HAL_PA_INIT_EVENT;
    } else {
        crate::println!("!! hal_tmos_task: task_id: {}, events: 0x{:04x}", task_id, events);

        // CHANNEL.publish_immediate(events);
        // assume all events are handled here
        return 0;
    }
}

/// Wrapper of BLEInit and HAL_Init
/// Returns a global task id.
pub fn init(
    config: Config,
) -> Result<(u8, Subscriber<'static, CriticalSectionRawMutex, TmosEvent, 4, 2, 2>), NonZeroU8> {
    use ffi::{bleConfig_t, BLE_LibInit, TMOS_ProcessEventRegister, TMOS_TimerInit, LL_TX_POWEER_6_DBM};

    const BLE_TX_NUM_EVENT: u8 = 1;
    const BLE_BUFF_NUM: u8 = 5;
    const BLE_BUFF_MAX_LEN: u16 = 27;
    const BLE_TX_POWER: u8 = LL_TX_POWEER_6_DBM;
    const PERIPHERAL_MAX_CONNECTION: u8 = 1;
    const CENTRAL_MAX_CONNECTION: u8 = 3;

    let mut cfg: bleConfig_t = unsafe { core::mem::zeroed() };

    cfg.MEMAddr = unsafe { BLE_HEAP.0.as_ptr() as u32 };
    cfg.MEMLen = HEAP_SIZE as _;

    cfg.BufMaxLen = BLE_BUFF_MAX_LEN;
    cfg.BufNumber = BLE_BUFF_NUM;
    cfg.TxNumEvent = BLE_TX_NUM_EVENT;
    cfg.TxPower = BLE_TX_POWER;

    // No SNV (SNVAddr, SNVBlock, SNVNum, readFlashCB, writeFlashCB)

    cfg.SelRTCClock = 0; // use LSE: ( 0 外部(32768Hz)，默认:1：内部(32000Hz)，2：内部(32768Hz)

    cfg.ConnectNumber = (PERIPHERAL_MAX_CONNECTION & 3) | (CENTRAL_MAX_CONNECTION << 2);

    cfg.srandCB = Some(srand); // tmos_rand will call this

    cfg.rcCB = None; // use LSE, no calibrate
    cfg.tsCB = Some(get_raw_temperature);

    // No need to set HAL_SLEEP(WakeUpTIme, sleepCB)

    cfg.MacAddr = config.mac_addr.0;

    unsafe {
        BLE_LibInit(&cfg)?;

        TMOS_TimerInit(core::ptr::null_mut())?;

        // regeister HAL tmos task
        let hal_task_id = TMOS_ProcessEventRegister(Some(hal_tmos_task));

        tmos_start_task(hal_task_id, HAL_REG_INIT_EVENT, HAL_TMOS_TASK_INTERVAL);

        if let Some(pa_config) = config.pa_config {
            // save to global PA_CONFIG
            configure_pa(pa_config);
            // trigger PA init immediately
            tmos_start_task(hal_task_id, HAL_PA_INIT_EVENT, 0);
        }

        Ok((hal_task_id, CHANNEL.subscriber().unwrap()))
    }
}

pub unsafe extern "C" fn srand() -> u32 {
    let systick = unsafe { &*pac::SYSTICK::PTR };
    systick.cnt.read().bits() as u32
}

pub unsafe extern "C" fn get_raw_temperature() -> u16 {
    let mut regs: [u8; 4] = [0; 4];

    let rb = &*pac::ADC::PTR;
    let sys = &*pac::SYS::PTR; // TODO: refine rb
    regs[0] = sys.tkey_cfg.read().bits();
    regs[1] = rb.tem_sensor.read().bits();
    regs[2] = rb.channel.read().bits();
    regs[3] = rb.cfg.read().bits();

    let peri = crate::peripherals::ADC::steal();
    let mut adc = crate::adc::Adc::new(peri, crate::adc::Config::for_temperature());
    let mut temp_sensor = adc.enable_temperature();
    let data = adc.read(&mut temp_sensor);

    core::mem::forget(adc);

    // restore regs
    sys.tkey_cfg.write(|w| w.bits(regs[0]));
    rb.tem_sensor.write(|w| w.bits(regs[1]));
    rb.channel.write(|w| w.bits(regs[2]));
    rb.cfg.write(|w| w.bits(regs[3]));

    data
}

/// "CH58x_BLE_LIB_V1.9"
pub fn lib_version() -> &'static str {
    unsafe {
        let version = CStr::from_ptr(ffi::VER_LIB.as_ptr());
        version.to_str().unwrap()
    }
}

/// Library format MAC Address, LSB first
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MacAddress([u8; 6]);

impl MacAddress {
    #[inline(always)]
    pub fn from_raw(addr: [u8; 6]) -> Self {
        MacAddress(addr)
    }

    /// Convert from human readable format, MSB first
    pub fn from_msb(addr: [u8; 6]) -> Self {
        MacAddress([addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]])
    }
}

impl core::fmt::Display for MacAddress {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let MacAddress(addr) = self;
        write!(
            f,
            "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
            addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]
        )
    }
}

impl From<[u8; 6]> for MacAddress {
    fn from(addr: [u8; 6]) -> Self {
        MacAddress::from_msb(addr)
    }
}
