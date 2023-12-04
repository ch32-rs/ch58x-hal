//! GATTServApp layer interface.
//!
//! This module implements the GATT Server App.

#![allow(
    non_camel_case_types,
    non_snake_case,
    non_upper_case_globals,
    unused_variables,
    non_camel_case_types
)]

use super::ffi::bStatus_t;
use super::gatt::GattAttribute;

// GATT Client Characteristic Configuration Bit Fields
pub const GATT_CLIENT_CFG_NOTIFY: u16 = 1;
pub const GATT_CLIENT_CFG_INDICATE: u16 = 2;

// All profile services bit fields
pub const GATT_ALL_SERVICES: u32 = 4294967295;

// GATT Encryption Key Size Limits
pub const GATT_MIN_ENCRYPT_KEY_SIZE: u8 = 7;
pub const GATT_MAX_ENCRYPT_KEY_SIZE: u8 = 16;

pub const INVALID_CONNHANDLE: u16 = 0xFFFF;

#[doc = " @brief   Callback function prototype to read an attribute value.\n\n @note    blePending can be returned ONLY for the following\n          read operations:\n          - Read Request: ATT_READ_REQ\n          - Read Blob Request: ATT_READ_BLOB_REQ\n\n @note    If blePending is returned then it's the responsibility of the application to respond to\n          ATT_READ_REQ and ATT_READ_BLOB_REQ message with ATT_READ_RSP and ATT_READ_BLOB_RSP\n          message respectively.\n\n @note    Payload 'pValue' used with ATT_READ_RSP and ATT_READ_BLOB_RSP must be allocated using GATT_bm_alloc().\n\n @param   connHandle - connection request was received on\n @param   pAttr - pointer to attribute\n @param   pValue - pointer to data to be read (to be returned)\n @param   pLen - length of data (to be returned)\n @param   offset - offset of the first octet to be read\n @param   maxLen - maximum length of data to be read\n @param   method - type of read message\n\n @return  SUCCESS: Read was successfully.<BR>\n          blePending: A response is pending for this client.<BR>\n          Error, otherwise: ref ATT_ERR_CODE_DEFINES.<BR>"]
pub type pfnGATTReadAttrCB_t = Option<
    unsafe extern "C" fn(
        conn_handle: u16,
        pAttr: *mut GattAttribute,
        pValue: *mut u8,
        pLen: *mut u16,
        offset: u16,
        max_len: u16,
        method: u8,
    ) -> u8,
>;
#[doc = " @brief   Callback function prototype to write an attribute value.\n\n @note    blePending can be returned ONLY for the following\n          write operations:\n          - Write Request: ATT_WRITE_REQ\n          - Write Command: ATT_WRITE_CMD\n          - Write Long: ATT_EXECUTE_WRITE_REQ\n          - Reliable Writes: Multiple ATT_PREPARE_WRITE_REQ followed by one final ATT_EXECUTE_WRITE_REQ\n\n @note    If blePending is returned then it's the responsibility of the application to 1) respond to\n          ATT_WRITE_REQ and ATT_EXECUTE_WRITE_REQ message with ATT_WRITE_RSP and ATT_EXECUTE_WRITE_RSP\n          message respectively, and 2) free each request payload 'pValue' using BM_free().\n\n @note    Write Command (ATT_WRITE_CMD) does NOT require a response message.\n\n @param   connHandle - connection request was received on\n @param   pAttr - pointer to attribute\n @param   pValue - pointer to data to be written\n @param   pLen - length of data\n @param   offset - offset of the first octet to be written\n @param   method - type of write message\n\n @return  SUCCESS: Write was successfully.<BR>\n          blePending: A response is pending for this client.<BR>\n          Error, otherwise: ref ATT_ERR_CODE_DEFINES.<BR>"]
pub type pfnGATTWriteAttrCB_t = Option<
    unsafe extern "C" fn(
        connHandle: u16,
        pAttr: *mut GattAttribute,
        pValue: *mut u8,
        len: u16,
        offset: u16,
        method: u8,
    ) -> u8,
>;
#[doc = " @brief   Callback function prototype to authorize a Read or Write operation\n          on a given attribute.\n\n @param   connHandle - connection request was received on\n @param   pAttr - pointer to attribute\n @param   opcode - request opcode (ATT_READ_REQ or ATT_WRITE_REQ)\n\n @return  SUCCESS: Operation authorized.<BR>\n          ATT_ERR_INSUFFICIENT_AUTHOR: Authorization required.<BR>"]
pub type pfnGATTAuthorizeAttrCB_t =
    Option<unsafe extern "C" fn(connHandle: u16, pAttr: *mut GattAttribute, opcode: u8) -> bStatus_t>;

#[doc = " GATT Structure for service callback functions - must be setup by the application\n and used when GATTServApp_RegisterService() is called."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gattServiceCBs_t {
    #[doc = "!< Read callback function pointer. pfnGATTReadAttrCB_t"]
    pub pfnReadAttrCB: pfnGATTReadAttrCB_t,
    #[doc = "!< Write callback function pointer"]
    pub pfnWriteAttrCB: pfnGATTWriteAttrCB_t,
    #[doc = "!< Authorization callback function pointer"]
    pub pfnAuthorizeAttrCB: pfnGATTAuthorizeAttrCB_t,
}

// save to add Default when init
#[doc = " GATT Structure for Client Characteristic Configuration."]
#[repr(C)]
#[derive(Debug, Copy, Clone, Default)]
pub struct gattCharCfg_t {
    #[doc = "!< Client connection handle"]
    pub connHandle: u16,
    #[doc = "!< Characteristic configuration value for this client"]
    pub value: u8,
}

pub struct GATTServApp;

impl GATTServApp {
    #[inline]
    pub fn register_service(
        attrs: &'static mut [GattAttribute],
        enc_key_size: u8,
        service_cbs: &'static gattServiceCBs_t,
    ) -> bStatus_t {
        unsafe { ffi::GATTServApp_RegisterService(attrs.as_mut_ptr(), attrs.len() as _, enc_key_size, service_cbs) }
    }

    #[inline]
    pub fn add_service(services: u32) -> bStatus_t {
        unsafe { ffi::GATTServApp_AddService(services) }
    }

    #[inline]
    pub fn deregister_service(handle: u16, p2pAttrs: *mut *mut GattAttribute) -> bStatus_t {
        unsafe { ffi::GATTServApp_DeregisterService(handle, p2pAttrs) }
    }

    #[inline]
    pub fn init_char_cfg(connHandle: u16, charCfgTbl: *mut gattCharCfg_t) {
        unsafe { ffi::GATTServApp_InitCharCfg(connHandle, charCfgTbl) }
    }

    #[inline]
    pub fn send_service_changed_ind(connHandle: u16, taskId: u8) -> bStatus_t {
        unsafe { ffi::GATTServApp_SendServiceChangedInd(connHandle, taskId) }
    }

    #[inline]
    pub fn read_char_cfg(connHandle: u16, charCfgTbl: *const gattCharCfg_t) -> u16 {
        unsafe { ffi::GATTServApp_ReadCharCfg(connHandle, charCfgTbl) }
    }

    #[inline]
    pub fn write_char_cfg(connHandle: u16, charCfgTbl: *mut gattCharCfg_t, value: u16) -> u8 {
        unsafe { ffi::GATTServApp_WriteCharCfg(connHandle, charCfgTbl, value) }
    }

    /// Process the client characteristic configuration write request for a given client.
    #[inline]
    pub fn process_ccc_write_req(
        connHandle: u16,
        pAttr: *mut GattAttribute,
        pValue: *mut u8,
        len: u16,
        offset: u16,
        validCfg: u16,
    ) -> bStatus_t {
        unsafe { ffi::GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len, offset, validCfg) }
    }
}

mod ffi {
    use super::*;
    extern "C" {
        #[doc = " @brief   Register a service's attribute list and callback functions with\n          the GATT Server Application.\n\n @param   pAttrs - Array of attribute records to be registered\n @param   numAttrs - Number of attributes in array\n @param   encKeySize - Minimum encryption key size required by service (7-16 bytes)\n @param   pServiceCBs - Service callback function pointers\n\n @return  SUCCESS: Service registered successfully.<BR>\n          INVALIDPARAMETER: Invalid service fields.<BR>\n          FAILURE: Not enough attribute handles available.<BR>\n          bleMemAllocError: Memory allocation error occurred.<BR>\n          bleInvalidRange: Encryption key size's out of range.<BR>"]
        pub fn GATTServApp_RegisterService(
            pAttrs: *mut GattAttribute,
            numAttrs: u16,
            encKeySize: u8,
            pServiceCBs: *const gattServiceCBs_t,
        ) -> bStatus_t;
        #[doc = " @brief   Add function for the GATT Service.\n\n @param   services - services to add. This is a bit map and can\n                     contain more than one service.\n\n @return  SUCCESS: Service added successfully.<BR>\n          INVALIDPARAMETER: Invalid service field.<BR>\n          FAILURE: Not enough attribute handles available.<BR>\n          bleMemAllocError: Memory allocation error occurred.<BR>"]
        pub fn GATTServApp_AddService(services: u32) -> bStatus_t;
        #[doc = " @brief   Deregister a service's attribute list and callback functions from\n          the GATT Server Application.\n\n @note    It's the caller's responsibility to free the service attribute\n          list returned from this API.\n\n @param   handle - handle of service to be deregistered\n @param   p2pAttrs - pointer to array of attribute records (to be returned)\n\n @return  SUCCESS: Service deregistered successfully.<BR>\n          FAILURE: Service not found.<BR>"]
        pub fn GATTServApp_DeregisterService(handle: u16, p2pAttrs: *mut *mut GattAttribute) -> bStatus_t;
        #[doc = " @brief   Initialize the client characteristic configuration table.\n\n @note    Each client has its own instantiation of the ClientCharacteristic Configuration.\n          Reads/Writes of the Client Characteristic Configuration only only affect the\n          configuration of that client.\n\n @param   connHandle - connection handle (0xFFFF for all connections).\n @param   charCfgTbl - client characteristic configuration table.\n\n @return  none"]
        pub fn GATTServApp_InitCharCfg(connHandle: u16, charCfgTbl: *mut gattCharCfg_t);
        #[doc = " @brief   Send out a Service Changed Indication.\n\n @param   connHandle - connection to use\n @param   taskId - task to be notified of confirmation\n\n @return  SUCCESS: Indication was sent successfully.<BR>\n          FAILURE: Service Changed attribute not found.<BR>\n          INVALIDPARAMETER: Invalid connection handle or request field.<BR>\n          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>\n          bleNotConnected: Connection is down.<BR>\n          blePending: A confirmation is pending with this client.<BR>"]
        pub fn GATTServApp_SendServiceChangedInd(connHandle: u16, taskId: u8) -> bStatus_t;
        #[doc = " @brief   Read the client characteristic configuration for a given client.\n\n @note    Each client has its own instantiation of the Client Characteristic Configuration.\n          Reads of the Client Characteristic Configuration only shows the configuration\n          for that client.\n\n @param   connHandle - connection handle.\n @param   charCfgTbl - client characteristic configuration table.\n\n @return  attribute value"]
        pub fn GATTServApp_ReadCharCfg(connHandle: u16, charCfgTbl: *const gattCharCfg_t) -> u16;
        #[doc = " @brief   Write the client characteristic configuration for a given client.\n\n @note    Each client has its own instantiation of the Client Characteristic Configuration.\n          Writes of the Client Characteristic Configuration only only affect the\n          configuration of that client.\n\n @param   connHandle - connection handle.\n @param   charCfgTbl - client characteristic configuration table.\n @param   value - attribute new value.\n\n @return  Success or Failure"]
        pub fn GATTServApp_WriteCharCfg(connHandle: u16, charCfgTbl: *mut gattCharCfg_t, value: u16) -> u8;
        #[doc = " @brief   Process the client characteristic configuration\n          write request for a given client.\n\n @param   connHandle - connection message was received on.\n @param   pAttr - pointer to attribute.\n @param   pValue - pointer to data to be written.\n @param   len - length of data.\n @param   offset - offset of the first octet to be written.\n @param   validCfg - valid configuration.\n\n @return  Success or Failure"]
        pub fn GATTServApp_ProcessCCCWriteReq(
            connHandle: u16,
            pAttr: *mut GattAttribute,
            pValue: *mut u8,
            len: u16,
            offset: u16,
            validCfg: u16,
        ) -> bStatus_t;
    }
}
