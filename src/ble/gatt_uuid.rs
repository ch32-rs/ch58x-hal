//! This file contains GATT Profile UUID types.
//!
//!  The 16-bit UUIDs are assigned by the Bluetooth SIG and published
//! in the Bluetooth Assigned Numbers page. Do not change these values.
//! Changing them will cause Bluetooth interoperability issues.
//!
//! Ref: https://www.bluetooth.com/specifications/assigned-numbers/

// UUID defined
extern "C" {
    // GATT Services
    /// 1800
    pub static gapServiceUUID: [u8; 2usize];
    /// 1801
    pub static gattServiceUUID: [u8; 2usize];

    // GATT Attribute Types
    pub static primaryServiceUUID: [u8; 2usize];
    pub static secondaryServiceUUID: [u8; 2usize];
    pub static includeUUID: [u8; 2usize];
    pub static characterUUID: [u8; 2usize];

    // GATT Characteristic Descriptors
    pub static charExtPropsUUID: [u8; 2usize];
    pub static charUserDescUUID: [u8; 2usize];
    pub static clientCharCfgUUID: [u8; 2usize];
    pub static servCharCfgUUID: [u8; 2usize];
    pub static charFormatUUID: [u8; 2usize];
    pub static charAggFormatUUID: [u8; 2usize];
    pub static validRangeUUID: [u8; 2usize];
    pub static extReportRefUUID: [u8; 2usize];
    pub static reportRefUUID: [u8; 2usize];

    // GATT Characteristic Types
    pub static deviceNameUUID: [u8; 2usize];
    pub static appearanceUUID: [u8; 2usize];
    pub static periPrivacyFlagUUID: [u8; 2usize];
    pub static reconnectAddrUUID: [u8; 2usize];
    pub static periConnParamUUID: [u8; 2usize];
    pub static serviceChangedUUID: [u8; 2usize];
    pub static centAddrResUUID: [u8; 2usize];
}
/**
 * GATT Services
 */
pub const GAP_SERVICE_UUID: u16 = 0x1800; // Generic Access Profile
pub const GATT_SERVICE_UUID: u16 = 0x1801; // Generic Attribute Profile

/**
 * GATT Declarations
 */
pub const GATT_PRIMARY_SERVICE_UUID: u16 = 0x2800; // Primary Service
pub const GATT_SECONDARY_SERVICE_UUID: u16 = 0x2801; // Secondary Service
pub const GATT_INCLUDE_UUID: u16 = 0x2802; // Include
pub const GATT_CHARACTER_UUID: u16 = 0x2803; // Characteristic

/**
 * GATT Descriptors
 */
pub const GATT_CHAR_EXT_PROPS_UUID: u16 = 0x2900; // Characteristic Extended Properties
pub const GATT_CHAR_USER_DESC_UUID: u16 = 0x2901; // Characteristic User Description
pub const GATT_CLIENT_CHAR_CFG_UUID: u16 = 0x2902; // Client Characteristic Configuration
pub const GATT_SERV_CHAR_CFG_UUID: u16 = 0x2903; // Server Characteristic Configuration
pub const GATT_CHAR_FORMAT_UUID: u16 = 0x2904; // Characteristic Presentation Format
pub const GATT_CHAR_AGG_FORMAT_UUID: u16 = 0x2905; // Characteristic Aggregate Format
pub const GATT_VALID_RANGE_UUID: u16 = 0x2906; // Valid Range
pub const GATT_EXT_REPORT_REF_UUID: u16 = 0x2907; // External Report Reference Descriptor
pub const GATT_REPORT_REF_UUID: u16 = 0x2908; // Report Reference Descriptor

/**
 * GATT Characteristics
 */
pub const DEVICE_NAME_UUID: u16 = 0x2A00; // Device Name
pub const APPEARANCE_UUID: u16 = 0x2A01; // Appearance
pub const PERI_PRIVACY_FLAG_UUID: u16 = 0x2A02; // Peripheral Privacy Flag
pub const RECONNECT_ADDR_UUID: u16 = 0x2A03; // Reconnection Address
pub const PERI_CONN_PARAM_UUID: u16 = 0x2A04; // Peripheral Preferred Connection Parameters
pub const SERVICE_CHANGED_UUID: u16 = 0x2A05; // Service Changed
pub const CENTRAL_ADDRESS_RESOLUTION_UUID: u16 = 0x2AA6; // Central Address Resolution
pub const RL_PRIVATE_ADDR_ONLY_UUID: u16 = 0x2AC9; // Resolvable Private Address Only
pub const ENC_DATA_KEY_MATERIAL_UUID: u16 = 0x2B88; // Encrypted Data Key Material
pub const LE_GATT_SEC_LEVELS_UUID: u16 = 0x2BF5; // LE GATT Security Levels

/**
 * GATT Service UUIDs
 */
pub const IMMEDIATE_ALERT_SERV_UUID: u16 = 0x1802; // Immediate Alert
pub const LINK_LOSS_SERV_UUID: u16 = 0x1803; // Link Loss
pub const TX_PWR_LEVEL_SERV_UUID: u16 = 0x1804; // Tx Power
pub const CURRENT_TIME_SERV_UUID: u16 = 0x1805; // Current Time Service
pub const REF_TIME_UPDATE_SERV_UUID: u16 = 0x1806; // Reference Time Update Service
pub const NEXT_DST_CHANGE_SERV_UUID: u16 = 0x1807; // Next DST Change Service
pub const GLUCOSE_SERV_UUID: u16 = 0x1808; // Glucose
pub const THERMOMETER_SERV_UUID: u16 = 0x1809; // Health Thermometer
pub const DEVINFO_SERV_UUID: u16 = 0x180A; // Device Information
pub const NWA_SERV_UUID: u16 = 0x180B; // Network Availability
pub const HEARTRATE_SERV_UUID: u16 = 0x180D; // Heart Rate
pub const PHONE_ALERT_STS_SERV_UUID: u16 = 0x180E; // Phone Alert Status Service
pub const BATT_SERV_UUID: u16 = 0x180F; // Battery Service
pub const BLOODPRESSURE_SERV_UUID: u16 = 0x1810; // Blood Pressure
pub const ALERT_NOTIF_SERV_UUID: u16 = 0x1811; // Alert Notification Service
pub const HID_SERV_UUID: u16 = 0x1812; // Human Interface Device
pub const SCAN_PARAM_SERV_UUID: u16 = 0x1813; // Scan Parameters
pub const RSC_SERV_UUID: u16 = 0x1814; // Running Speed and Cadence
pub const CSC_SERV_UUID: u16 = 0x1816; // Cycling Speed and Cadence
pub const CYCPWR_SERV_UUID: u16 = 0x1818; // Cycling Power
pub const LOC_NAV_SERV_UUID: u16 = 0x1819; // Location and Navigation

/**
 * GATT Characteristic UUIDs
 */
pub const ALERT_LEVEL_UUID: u16 = 0x2A06; // Alert Level
pub const TX_PWR_LEVEL_UUID: u16 = 0x2A07; // Tx Power Level
pub const DATE_TIME_UUID: u16 = 0x2A08; // Date Time
pub const DAY_OF_WEEK_UUID: u16 = 0x2A09; // Day of Week
pub const DAY_DATE_TIME_UUID: u16 = 0x2A0A; // Day Date Time
pub const EXACT_TIME_256_UUID: u16 = 0x2A0C; // Exact Time 256
pub const DST_OFFSET_UUID: u16 = 0x2A0D; // DST Offset
pub const TIME_ZONE_UUID: u16 = 0x2A0E; // Time Zone
pub const LOCAL_TIME_INFO_UUID: u16 = 0x2A0F; // Local Time Information
pub const TIME_WITH_DST_UUID: u16 = 0x2A11; // Time with DST
pub const TIME_ACCURACY_UUID: u16 = 0x2A12; // Time Accuracy
pub const TIME_SOURCE_UUID: u16 = 0x2A13; // Time Source
pub const REF_TIME_INFO_UUID: u16 = 0x2A14; // Reference Time Information
pub const TIME_UPDATE_CTRL_PT_UUID: u16 = 0x2A16; // Time Update Control Point
pub const TIME_UPDATE_STATE_UUID: u16 = 0x2A17; // Time Update State
pub const GLUCOSE_MEAS_UUID: u16 = 0x2A18; // Glucose Measurement
pub const BATT_LEVEL_UUID: u16 = 0x2A19; // Battery Level
pub const TEMP_MEAS_UUID: u16 = 0x2A1C; // Temperature Measurement
pub const TEMP_TYPE_UUID: u16 = 0x2A1D; // Temperature Type
pub const IMEDIATE_TEMP_UUID: u16 = 0x2A1E; // Intermediate Temperature
pub const MEAS_INTERVAL_UUID: u16 = 0x2A21; // Measurement Interval
pub const BOOT_KEY_INPUT_UUID: u16 = 0x2A22; // Boot Keyboard Input Report
pub const SYSTEM_ID_UUID: u16 = 0x2A23; // System ID
pub const MODEL_NUMBER_UUID: u16 = 0x2A24; // Model Number String
pub const SERIAL_NUMBER_UUID: u16 = 0x2A25; // Serial Number String
pub const FIRMWARE_REV_UUID: u16 = 0x2A26; // Firmware Revision String
pub const HARDWARE_REV_UUID: u16 = 0x2A27; // Hardware Revision String
pub const SOFTWARE_REV_UUID: u16 = 0x2A28; // Software Revision String
pub const MANUFACTURER_NAME_UUID: u16 = 0x2A29; // Manufacturer Name String
pub const IEEE_11073_CERT_DATA_UUID: u16 = 0x2A2A; // IEEE 11073-20601 Regulatory Certification Data List
pub const CURRENT_TIME_UUID: u16 = 0x2A2B; // Current Time
pub const SCAN_REFRESH_UUID: u16 = 0x2A31; // Scan Refresh
pub const BOOT_KEY_OUTPUT_UUID: u16 = 0x2A32; // Boot Keyboard Output Report
pub const BOOT_MOUSE_INPUT_UUID: u16 = 0x2A33; // Boot Mouse Input Report
pub const GLUCOSE_CONTEXT_UUID: u16 = 0x2A34; // Glucose Measurement Context
pub const BLOODPRESSURE_MEAS_UUID: u16 = 0x2A35; // Blood Pressure Measurement
pub const IMEDIATE_CUFF_PRESSURE_UUID: u16 = 0x2A36; // Intermediate Cuff Pressure
pub const HEARTRATE_MEAS_UUID: u16 = 0x2A37; // Heart Rate Measurement
pub const BODY_SENSOR_LOC_UUID: u16 = 0x2A38; // Body Sensor Location
pub const HEARTRATE_CTRL_PT_UUID: u16 = 0x2A39; // Heart Rate Control Point
pub const NETWORK_AVAIL_UUID: u16 = 0x2A3E; // Network Availability
pub const ALERT_STATUS_UUID: u16 = 0x2A3F; // Alert Status
pub const RINGER_CTRL_PT_UUID: u16 = 0x2A40; // Ringer Control Point
pub const RINGER_SETTING_UUID: u16 = 0x2A41; // Ringer Setting
pub const ALERT_CAT_ID_BMASK_UUID: u16 = 0x2A42; // Alert Category ID Bit Mask
pub const ALERT_CAT_ID_UUID: u16 = 0x2A43; // Alert Category ID
pub const ALERT_NOTIF_CTRL_PT_UUID: u16 = 0x2A44; // Alert Notification Control Point
pub const UNREAD_ALERT_STATUS_UUID: u16 = 0x2A45; // Unread Alert Status
pub const NEW_ALERT_UUID: u16 = 0x2A46; // New Alert
pub const SUP_NEW_ALERT_CAT_UUID: u16 = 0x2A47; // Supported New Alert Category
pub const SUP_UNREAD_ALERT_CAT_UUID: u16 = 0x2A48; // Supported Unread Alert Category
pub const BLOODPRESSURE_FEATURE_UUID: u16 = 0x2A49; // Blood Pressure Feature
pub const HID_INFORMATION_UUID: u16 = 0x2A4A; // HID Information
pub const REPORT_MAP_UUID: u16 = 0x2A4B; // Report Map
pub const HID_CTRL_PT_UUID: u16 = 0x2A4C; // HID Control Point
pub const REPORT_UUID: u16 = 0x2A4D; // Report
pub const PROTOCOL_MODE_UUID: u16 = 0x2A4E; // Protocol Mode
pub const SCAN_INTERVAL_WINDOW_UUID: u16 = 0x2A4F; // Scan Interval Window
pub const PNP_ID_UUID: u16 = 0x2A50; // PnP ID
pub const GLUCOSE_FEATURE_UUID: u16 = 0x2A51; // Glucose Feature
pub const RECORD_CTRL_PT_UUID: u16 = 0x2A52; // Record Access Control Point
pub const RSC_MEAS_UUID: u16 = 0x2A53; // RSC Measurement
pub const RSC_FEATURE_UUID: u16 = 0x2A54; // RSC Feature
pub const SC_CTRL_PT_UUID: u16 = 0x2A55; // SC Control Point
pub const CSC_MEAS_UUID: u16 = 0x2A5B; // CSC Measurement
pub const CSC_FEATURE_UUID: u16 = 0x2A5C; // CSC Feature
pub const SENSOR_LOC_UUID: u16 = 0x2A5D; // Sensor Location
pub const CYCPWR_MEAS_UUID: u16 = 0x2A63; // Cycling Power Measurement
pub const CYCPWR_VECTOR_UUID: u16 = 0x2A64; // Cycling Power Vector
pub const CYCPWR_FEATURE_UUID: u16 = 0x2A65; // Cycling Power Feature
pub const CYCPWR_CTRL_PT_UUID: u16 = 0x2A66; // Cycling Power Control Point
pub const LOC_SPEED_UUID: u16 = 0x2A67; // Location and Speed
pub const NAV_UUID: u16 = 0x2A68; // Navigation
pub const POS_QUALITY_UUID: u16 = 0x2A69; // Position Quality
pub const LN_FEATURE_UUID: u16 = 0x2A6A; // LN Feature
pub const LN_CTRL_PT_UUID: u16 = 0x2A6B; // LN Control Point
pub const ELE_UUID: u16 = 0x2A6C; // Elevation
pub const PRESSURE_UUID: u16 = 0x2A6D; // Pressure
pub const TEMP_UUID: u16 = 0x2A6E; // Temperature
pub const HUMI_UUID: u16 = 0x2A6F; // Humidity
pub const TRUE_WIND_SPEED_UUID: u16 = 0x2A70; // True Wind Speed
pub const TRUE_WIND_DIRECTION_UUID: u16 = 0x2A71; // True Wind Direction
pub const URI_UUID: u16 = 0x2AB6; // URI
pub const MEDIA_STATE_UUID: u16 = 0x2BA3; // Media State
pub const MEDIA_CTRL_PT_UUID: u16 = 0x2BA4; // Media Control Point
pub const MEDIA_CTRL_PT_OS_UUID: u16 = 0x2BA5; // Media Control Point Opcodes Supported
pub const CALL_STATE_UUID: u16 = 0x2BBD; // Call State
pub const CALL_CTRL_PT_UUID: u16 = 0x2BBE; // Call Control Point
pub const CALL_CTRL_PT_OO_UUID: u16 = 0x2BBF; // Call Control Point Optional Opcodes
pub const TERM_REASON_UUID: u16 = 0x2BC0; // Termination Reason
pub const INCOMING_CALL_UUID: u16 = 0x2BC1; // Incoming Call
pub const MUTE_UUID: u16 = 0x2BC3; // Mute
pub const ESL_ADDR_UUID: u16 = 0x2BF6; // ESL Address
pub const AP_SYNC_KEY_MATERIAL_UUID: u16 = 0x2BF7; // AP Sync Key Material
pub const ESL_RSP_KEY_MATERIAL_UUID: u16 = 0x2BF8; // ESL Response Key Material
pub const ESL_CURR_ABS_TIME_UUID: u16 = 0x2BF9; // ESL Current Absolute Time
pub const ESL_DISPLAY_INFO_UUID: u16 = 0x2BFA; // ESL Display Information
pub const ESL_IMAGE_INFO_UUID: u16 = 0x2BFB; // ESL Image Information
pub const ESL_SENSOR_INFO_UUID: u16 = 0x2BFC; // ESL Sensor Information
pub const ESL_LED_INFO_UUID: u16 = 0x2BFD; // ESL LED Information
pub const ESL_CTL_POINT_UUID: u16 = 0x2BFE; // ESL Control Point

/**
 * GATT Unit UUIDs
 */
pub const GATT_UNITLESS_UUID: u16 = 0x2700; // unitless
pub const GATT_UNIT_LENGTH_METER_UUID: u16 = 0x2701; // m, m
pub const GATT_UNIT_MASS_KGRAM_UUID: u16 = 0x2702; // kg, kg
pub const GATT_UNIT_TIME_SECOND_UUID: u16 = 0x2703; // s, s
pub const GATT_UNIT_ELECTRIC_CURRENT_A_UUID: u16 = 0x2704; // A, A
pub const GATT_UNIT_THERMODYN_TEMP_K_UUID: u16 = 0x2705; // K, K
pub const GATT_UNIT_AMOUNT_SUBSTANCE_M_UUID: u16 = 0x2706; // mol, mol
pub const GATT_UNIT_LUMINOUS_INTENSITY_C_UUID: u16 = 0x2707; // cd, cd

pub const GATT_UNIT_AREA_SQ_MTR_UUID: u16 = 0x2710; // m^2, m^2
pub const GATT_UNIT_VOLUME_CUBIC_MTR_UUID: u16 = 0x2711; // m^3, m^3
pub const GATT_UNIT_VELOCITY_MPS_UUID: u16 = 0x2712; // m/s, m s^-1
pub const GATT_UNIT_ACCELERATION_MPS_SQ_UUID: u16 = 0x2713; // m/s^2, m s^-2
pub const GATT_UNIT_WAVENUMBER_RM_UUID: u16 = 0x2714; // ? m^-1
pub const GATT_UNIT_DENSITY_KGPCM_UUID: u16 = 0x2715; // p, kg m^-3
pub const GATT_UNIT_SURFACE_DENSITY_KGPSM_UUID: u16 = 0x2716; // pA, kg m^-2
pub const GATT_UNIT_SPECIFIC_VOLUME_CMPKG_UUID: u16 = 0x2717; // v, m^3 kg^-1
pub const GATT_UNIT_CURRENT_DENSITY_APSM_UUID: u16 = 0x2718; // j, A m^-2
pub const GATT_UNIT_MAG_FIELD_STRENGTH_UUID: u16 = 0x2719; // H, A m
pub const GATT_UNIT_AMOUNT_CONC_MPCM_UUID: u16 = 0x271A; // c, mol m^-3
pub const GATT_UNIT_MASS_CONC_KGPCM_UUID: u16 = 0x271B; // c, kg m^-3
pub const GATT_UNIT_LUMINANCE_CPSM_UUID: u16 = 0x271C; // Lv, cd m^-2
pub const GATT_UNIT_REFRACTIVE_INDEX_UUID: u16 = 0x271D; // n, 1
pub const GATT_UNIT_RELATIVE_PERMEABLILTY_UUID: u16 = 0x271E; // u, 1
pub const GATT_UNIT_PLANE_ANGLE_RAD_UUID: u16 = 0x2720; // rad, m m-1
pub const GATT_UNIT_SOLID_ANGLE_STERAD_UUID: u16 = 0x2721; // sr, m2 m-2
pub const GATT_UNIT_FREQUENCY_HTZ_UUID: u16 = 0x2722; // Hz, s-1
pub const GATT_UNIT_FORCE_NEWTON_UUID: u16 = 0x2723; // N, m kg s-2
pub const GATT_UNIT_PRESSURE_PASCAL_UUID: u16 = 0x2724; // Pa, N/m2 = m2 kg s-2
pub const GATT_UNIT_ENERGY_JOULE_UUID: u16 = 0x2725; // J, N m = m2 kg s-2
pub const GATT_UNIT_POWER_WATT_UUID: u16 = 0x2726; // W, J/s = m2 kg s-3
pub const GATT_UNIT_E_CHARGE_C_UUID: u16 = 0x2727; // C, sA
pub const GATT_UNIT_E_POTENTIAL_DIF_V_UUID: u16 = 0x2728; // V, W/A = m2 kg s-3 A-1

pub const GATT_UNIT_CELSIUS_TEMP_DC_UUID: u16 = 0x272F; // oC, t/oC = T/K - 273.15

pub const GATT_UNIT_TIME_MINUTE_UUID: u16 = 0x2760; // min, 60 s
pub const GATT_UNIT_TIME_HOUR_UUID: u16 = 0x2761; // h, 3600 s
pub const GATT_UNIT_TIME_DAY_UUID: u16 = 0x2762; // d, 86400 s
pub const GATT_UNIT_PLANE_ANGLE_DEGREE_UUID: u16 = 0x2763; // o, (pi/180) rad
pub const GATT_UNIT_PLANE_ANGLE_MINUTE_UUID: u16 = 0x2764; // ', (pi/10800) rad
pub const GATT_UNIT_PLANE_ANGLE_SECOND_UUID: u16 = 0x2765; // '', (pi/648000) rad
pub const GATT_UNIT_AREA_HECTARE_UUID: u16 = 0x2766; // ha, 10^4 m^2
pub const GATT_UNIT_VOLUME_LITRE_UUID: u16 = 0x2767; // l, 10^-3 m^3
pub const GATT_UNIT_MASS_TONNE_UUID: u16 = 0x2768; // t, 10^3 kg

pub const GATT_UINT_LENGTH_YARD_UUID: u16 = 0x27A0; // yd, 0.9144 m
pub const GATT_UNIT_LENGTH_PARSEC_UUID: u16 = 0x27A1; // pc, 3.085678 ?1016 m
pub const GATT_UNIT_LENGTH_INCH_UUID: u16 = 0x27A2; // in, 0.0254 m
pub const GATT_UNIT_LENGTH_FOOT_UUID: u16 = 0x27A3; // ft, 0.3048 m
pub const GATT_UNIT_LENGTH_MILE_UUID: u16 = 0x27A4; // mi, 1609.347 m
pub const GATT_UNIT_PRESSURE_PFPSI_UUID: u16 = 0x27A5; // psi, 6.894757 ?103 Pa
pub const GATT_UNIT_VELOCITY_KMPH_UUID: u16 = 0x27A6; // km/h, 0.2777778 m^s-1
pub const GATT_UNIT_VELOCITY_MPH_UUID: u16 = 0x27A7; // mi/h, 0.44704 m^ s-1
pub const GATT_UNIT_ANGULAR_VELOCITY_RPM_UUID: u16 = 0x27A8; // r/min, 0.1047198 rad s-1
pub const GATT_UNIT_ENERGY_GCAL_UUID: u16 = 0x27A9; // energy (gram calorie)
pub const GATT_UNIT_ENERGY_KCAL_UUID: u16 = 0x27AA; // kcal, 4190.02 J
pub const GATT_UNIT_ENERGY_KWH_UUID: u16 = 0x27AB; // kWh, 3600000 J
pub const GATT_UNIT_THERMODYN_TEMP_DF_UUID: u16 = 0x27AC; // oF, t/oF = T/K ?1.8 - 459.67
pub const GATT_UNIT_PERCENTAGE_UUID: u16 = 0x27AD; // percentage,%
pub const GATT_UNIT_PER_MILE_UUID: u16 = 0x27AE; // per mille
pub const GATT_UNIT_PERIOD_BPM_UUID: u16 = 0x27AF; // period (beats per minute),BPM
pub const GATT_UNIT_E_CHARGE_AH_UUID: u16 = 0x27B0; // electric charge (ampere hours)
pub const GATT_UNIT_MASS_DENSITY_MGPD_UUID: u16 = 0x27B1; // mass density (milligram per decilitre)
pub const GATT_UNIT_MASS_DENSITY_MMPL_UUID: u16 = 0x27B2; // mass density (millimole per litre)
pub const GATT_UNIT_TIME_YEAR_UUID: u16 = 0x27B3; // time (year)
pub const GATT_UNIT_TIME_MONTH_UUID: u16 = 0x27B4; // time (month)
