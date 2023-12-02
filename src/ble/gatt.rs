#![allow(non_snake_case, non_camel_case_types)]

use super::ffi::{bStatus_t, tmos_event_hdr_t};

//  ATT Error Codes
/// The attribute handle given was not valid on this server
pub const ATT_ERR_INVALID_HANDLE: u8 = 0x01;

/// The attribute cannot be read
pub const ATT_ERR_READ_NOT_PERMITTED: u8 = 0x02;

/// The attribute cannot be written
pub const ATT_ERR_WRITE_NOT_PERMITTED: u8 = 0x03;

/// The attribute PDU was invalid
pub const ATT_ERR_INVALID_PDU: u8 = 0x04;

/// The attribute requires authentication before it can be read or written
pub const ATT_ERR_INSUFFICIENT_AUTHEN: u8 = 0x05;

/// Attribute server does not support the request received from the client
pub const ATT_ERR_UNSUPPORTED_REQ: u8 = 0x06;

/// Offset specified was past the end of the attribute
pub const ATT_ERR_INVALID_OFFSET: u8 = 0x07;

/// The attribute requires authorization before it can be read or written
pub const ATT_ERR_INSUFFICIENT_AUTHOR: u8 = 0x08;

/// Too many prepare writes have been queued
pub const ATT_ERR_PREPARE_QUEUE_FULL: u8 = 0x09;

/// No attribute found within the given attribute handle range
pub const ATT_ERR_ATTR_NOT_FOUND: u8 = 0x0a;

/// The attribute cannot be read using the Read Blob Request
pub const ATT_ERR_ATTR_NOT_LONG: u8 = 0x0b;

/// The Encryption Key Size used for encrypting this link is insufficient
pub const ATT_ERR_INSUFFICIENT_KEY_SIZE: u8 = 0x0c;

/// The attribute value length is invalid for the operation
pub const ATT_ERR_INVALID_VALUE_SIZE: u8 = 0x0d;

/// The attribute request that was requested has encountered an error that was very unlikely, and therefore could not be completed as requested
pub const ATT_ERR_UNLIKELY: u8 = 0x0e;

/// The attribute requires encryption before it can be read or written
pub const ATT_ERR_INSUFFICIENT_ENCRYPT: u8 = 0x0f;

/// The attribute type is not a supported grouping attribute as defined by a higher layer specification
pub const ATT_ERR_UNSUPPORTED_GRP_TYPE: u8 = 0x10;

/// Insufficient Resources to complete the request
pub const ATT_ERR_INSUFFICIENT_RESOURCES: u8 = 0x11;

/// The attribute value is invalid for the operation
pub const ATT_ERR_INVALID_VALUE: u8 = 0x80;

// tmos event
#[doc = " GATT tmos GATT_MSG_EVENT message format. This message is used to forward an\n incoming attribute protocol/profile message up to upper layer application."]
#[repr(C)]
#[derive(Copy, Clone)]
pub struct gattMsgEvent_t {
    #[doc = "!< GATT_MSG_EVENT and status"]
    pub hdr: tmos_event_hdr_t,
    #[doc = "!< Connection message was received on"]
    pub connHandle: u16,
    #[doc = "!< Type of message"]
    pub method: u8,
    #[doc = "!< Attribute protocol/profile message"]
    pub msg: gattMsg_t,
}

#[doc = " GATT Message format. It's a union of all attribute protocol/profile messages\n and locally-generated events used between the attribute protocol/profile and\n upper layer application."]
#[repr(C)]
#[derive(Copy, Clone)]
pub union gattMsg_t {
    #[doc = "!< ATT Exchange MTU Request"]
    pub exchangeMTUReq: attExchangeMTUReq_t,
    #[doc = "!< ATT Find Information Request"]
    pub findInfoReq: attFindInfoReq_t,
    #[doc = "!< ATT Find By Type Value Request"]
    pub findByTypeValueReq: attFindByTypeValueReq_t,
    #[doc = "!< ATT Read By Type Request"]
    pub readByTypeReq: attReadByTypeReq_t,
    #[doc = "!< ATT Read Request"]
    pub readReq: attReadReq_t,
    #[doc = "!< ATT Read Blob Request"]
    pub readBlobReq: attReadBlobReq_t,
    #[doc = "!< ATT Read Multiple Request"]
    pub readMultiReq: attReadMultiReq_t,
    #[doc = "!< ATT Read By Group Type Request"]
    pub readByGrpTypeReq: attReadByGrpTypeReq_t,
    #[doc = "!< ATT Write Request"]
    pub writeReq: attWriteReq_t,
    #[doc = "!< ATT Prepare Write Request"]
    pub prepareWriteReq: attPrepareWriteReq_t,
    #[doc = "!< ATT Execute Write Request"]
    pub executeWriteReq: attExecuteWriteReq_t,
    #[doc = "!< GATT Find By Type Value Request"]
    pub gattFindByTypeValueReq: gattFindByTypeValueReq_t,
    #[doc = "!< GATT Read By Type Request"]
    pub gattReadByTypeReq: gattReadByTypeReq_t,
    #[doc = "!< GATT Long Write Request"]
    pub gattWriteLongReq: gattWriteLongReq_t,
    #[doc = "!< GATT Reliable Writes Request"]
    pub gattReliableWritesReq: gattReliableWritesReq_t,
    #[doc = "!< ATT Error Response"]
    pub errorRsp: attErrorRsp_t,
    #[doc = "!< ATT Exchange MTU Response"]
    pub exchangeMTURsp: attExchangeMTURsp_t,
    #[doc = "!< ATT Find Information Response"]
    pub findInfoRsp: attFindInfoRsp_t,
    #[doc = "!< ATT Find By Type Value Response"]
    pub findByTypeValueRsp: attFindByTypeValueRsp_t,
    #[doc = "!< ATT Read By Type Response"]
    pub readByTypeRsp: attReadByTypeRsp_t,
    #[doc = "!< ATT Read Response"]
    pub readRsp: attReadRsp_t,
    #[doc = "!< ATT Read Blob Response"]
    pub readBlobRsp: attReadBlobRsp_t,
    #[doc = "!< ATT Read Multiple Response"]
    pub readMultiRsp: attReadMultiRsp_t,
    #[doc = "!< ATT Read By Group Type Response"]
    pub readByGrpTypeRsp: attReadByGrpTypeRsp_t,
    #[doc = "!< ATT Prepare Write Response"]
    pub prepareWriteRsp: attPrepareWriteRsp_t,
    #[doc = "!< ATT Handle Value Notification"]
    pub handleValueNoti: attHandleValueNoti_t,
    #[doc = "!< ATT Handle Value Indication"]
    pub handleValueInd: attHandleValueInd_t,
    #[doc = "!< ATT Flow Control Violated Event"]
    pub flowCtrlEvt: attFlowCtrlViolatedEvt_t,
    #[doc = "!< ATT MTU Updated Event"]
    pub mtuEvt: attMtuUpdatedEvt_t,
}

#[doc = " Error Response format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attErrorRsp_t {
    #[doc = "!< Request that generated this error response"]
    pub reqOpcode: u8,
    #[doc = "!< Attribute handle that generated error response"]
    pub handle: u16,
    #[doc = "!< Reason why the request has generated error response"]
    pub errCode: u8,
}

#[doc = " Exchange MTU Request format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attExchangeMTUReq_t {
    #[doc = "!< Client receive MTU size"]
    pub clientRxMTU: u16,
}
#[doc = " Exchange MTU Response format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attExchangeMTURsp_t {
    #[doc = "!< Server receive MTU size"]
    pub serverRxMTU: u16,
}
#[doc = " Find Information Request format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attFindInfoReq_t {
    #[doc = "!< First requested handle number (must be first field)"]
    pub startHandle: u16,
    #[doc = "!< Last requested handle number"]
    pub endHandle: u16,
}
#[doc = " Find Information Response format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attFindInfoRsp_t {
    #[doc = "!< Number of attribute handle-UUID pairs found"]
    pub numInfo: u16,
    #[doc = "!< Format of information data"]
    pub format: u8,
    #[doc = "!< Information data whose format is determined by format field (4 to ATT_MTU_SIZE-2)"]
    pub pInfo: *mut u8,
}
#[doc = " Find By Type Value Request format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attFindByTypeValueReq_t {
    #[doc = "!< First requested handle number (must be first field)"]
    pub startHandle: u16,
    #[doc = "!< Last requested handle number"]
    pub endHandle: u16,
    #[doc = "!< 2-octet UUID to find"]
    pub type_: attAttrBtType_t,
    #[doc = "!< Length of value"]
    pub len: u16,
    #[doc = "!< Attribute value to find (0 to ATT_MTU_SIZE-7)"]
    pub pValue: *mut u8,
}
#[doc = " Find By Type Value Response format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attFindByTypeValueRsp_t {
    #[doc = "!< Number of handles information found"]
    pub numInfo: u16,
    #[doc = "!< List of 1 or more handles information (4 to ATT_MTU_SIZE-1)"]
    pub pHandlesInfo: *mut u8,
}
#[doc = " Read By Type Request format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attReadByTypeReq_t {
    #[doc = "!< First requested handle number (must be first field)"]
    pub startHandle: u16,
    #[doc = "!< Last requested handle number"]
    pub endHandle: u16,
    #[doc = "!< Requested type (2 or 16 octet UUID)"]
    pub type_: attAttrType_t,
}
#[doc = " Read By Type Response format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attReadByTypeRsp_t {
    #[doc = "!< Number of attribute handle-UUID pairs found"]
    pub numPairs: u16,
    #[doc = "!< Size of each attribute handle-value pair"]
    pub len: u16,
    #[doc = "!< List of 1 or more attribute handle-value pairs (2 to ATT_MTU_SIZE-2)"]
    pub pDataList: *mut u8,
}
#[doc = " Read Request format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attReadReq_t {
    #[doc = "!< Handle of the attribute to be read (must be first field)"]
    pub handle: u16,
}
#[doc = " Read Response format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attReadRsp_t {
    #[doc = "!< Length of value"]
    pub len: u16,
    #[doc = "!< Value of the attribute with the handle given (0 to ATT_MTU_SIZE-1)"]
    pub pValue: *mut u8,
}
#[doc = " Read Blob Req format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attReadBlobReq_t {
    #[doc = "!< Handle of the attribute to be read (must be first field)"]
    pub handle: u16,
    #[doc = "!< Offset of the first octet to be read"]
    pub offset: u16,
}
#[doc = " Read Blob Response format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attReadBlobRsp_t {
    #[doc = "!< Length of value"]
    pub len: u16,
    #[doc = "!< Part of the value of the attribute with the handle given (0 to ATT_MTU_SIZE-1)"]
    pub pValue: *mut u8,
}
#[doc = " Read Multiple Request format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attReadMultiReq_t {
    #[doc = "!< Set of two or more attribute handles (4 to ATT_MTU_SIZE-1) - must be first field"]
    pub pHandles: *mut u8,
    #[doc = "!< Number of attribute handles"]
    pub numHandles: u16,
}
#[doc = " Read Multiple Response format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attReadMultiRsp_t {
    #[doc = "!< Length of values"]
    pub len: u16,
    #[doc = "!< Set of two or more values (0 to ATT_MTU_SIZE-1)"]
    pub pValues: *mut u8,
}
#[doc = " Read By Group Type Request format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attReadByGrpTypeReq_t {
    #[doc = "!< First requested handle number (must be first field)"]
    pub startHandle: u16,
    #[doc = "!< Last requested handle number"]
    pub endHandle: u16,
    #[doc = "!< Requested group type (2 or 16 octet UUID)"]
    pub type_: attAttrType_t,
}
#[doc = " Read By Group Type Response format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attReadByGrpTypeRsp_t {
    #[doc = "!< Number of attribute handle, end group handle and value sets found"]
    pub numGrps: u16,
    #[doc = "!< Length of each attribute handle, end group handle and value set"]
    pub len: u16,
    #[doc = "!< List of 1 or more attribute handle, end group handle and value (4 to ATT_MTU_SIZE-2)"]
    pub pDataList: *mut u8,
}
#[doc = " Write Request format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attWriteReq_t {
    #[doc = "!< Handle of the attribute to be written (must be first field)"]
    pub handle: u16,
    #[doc = "!< Length of value"]
    pub len: u16,
    #[doc = "!< Value of the attribute to be written (0 to ATT_MTU_SIZE-3)"]
    pub pValue: *mut u8,
    #[doc = "!< Authentication Signature status (not included (0), valid (1), invalid (2))"]
    pub sig: u8,
    #[doc = "!< Command Flag"]
    pub cmd: u8,
}
#[doc = " Prepare Write Request format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attPrepareWriteReq_t {
    #[doc = "!< Handle of the attribute to be written (must be first field)"]
    pub handle: u16,
    #[doc = "!< Offset of the first octet to be written"]
    pub offset: u16,
    #[doc = "!< Length of value"]
    pub len: u16,
    #[doc = "!< Part of the value of the attribute to be written (0 to ATT_MTU_SIZE-5) - must be allocated"]
    pub pValue: *mut u8,
}
#[doc = " Prepare Write Response format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attPrepareWriteRsp_t {
    #[doc = "!< Handle of the attribute that has been read"]
    pub handle: u16,
    #[doc = "!< Offset of the first octet to be written"]
    pub offset: u16,
    #[doc = "!< Length of value"]
    pub len: u16,
    #[doc = "!< Part of the value of the attribute to be written (0 to ATT_MTU_SIZE-5)"]
    pub pValue: *mut u8,
}
#[doc = " Execute Write Request format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attExecuteWriteReq_t {
    #[doc = "!< 0x00 - cancel all prepared writes.\n!< 0x01 - immediately write all pending prepared values."]
    pub flags: u8,
}
#[doc = " Handle Value Notification format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attHandleValueNoti_t {
    #[doc = "!< Handle of the attribute that has been changed (must be first field)"]
    pub handle: u16,
    #[doc = "!< Length of value"]
    pub len: u16,
    #[doc = "!< Current value of the attribute (0 to ATT_MTU_SIZE-3)"]
    pub pValue: *mut u8,
}
#[doc = " Handle Value Indication format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attHandleValueInd_t {
    #[doc = "!< Handle of the attribute that has been changed (must be first field)"]
    pub handle: u16,
    #[doc = "!< Length of value"]
    pub len: u16,
    #[doc = "!< Current value of the attribute (0 to ATT_MTU_SIZE-3)"]
    pub pValue: *mut u8,
}
#[doc = " ATT Flow Control Violated Event message format.  This message is sent to the\n app by the local ATT Server or Client when a sequential ATT Request-Response\n or Indication-Confirmation protocol flow control is violated for a connection.\n All subsequent ATT Requests and Indications received by the local ATT Server\n and Client respectively will be dropped.\n\n This message is to inform the app (that has registered with GAP by calling\n GAP_RegisterForMsgs()) in case it wants to drop the connection."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attFlowCtrlViolatedEvt_t {
    #[doc = "!< opcode of message that caused flow control violation"]
    pub opcode: u8,
    #[doc = "!< opcode of pending message"]
    pub pendingOpcode: u8,
}
#[doc = " ATT MTU Updated Event message format.  This message is sent to the app\n by the local ATT Server or Client when the ATT MTU size is updated for a\n connection. The default ATT MTU size is 23 octets.\n\n This message is to inform the app (that has registered with GAP by calling\n GAP_RegisterForMsgs()) about the new ATT MTU size negotiated for a connection."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attMtuUpdatedEvt_t {
    #[doc = "!< new MTU size"]
    pub MTU: u16,
}

#[doc = " GATT Find By Type Value Request format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gattFindByTypeValueReq_t {
    #[doc = "!< First requested handle number (must be first field)"]
    pub startHandle: u16,
    #[doc = "!< Last requested handle number"]
    pub endHandle: u16,
    #[doc = "!< Primary service UUID value (2 or 16 octets)"]
    pub value: attAttrType_t,
}
#[doc = " GATT Read By Type Request format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gattReadByTypeReq_t {
    #[doc = "!< Whether this is a GATT Discover Characteristics by UUID sub-procedure"]
    pub discCharsByUUID: u8,
    #[doc = "!< Read By Type Request"]
    pub req: attReadByTypeReq_t,
}
#[doc = " GATT Write Long Request format. Do not change the order of the members."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gattWriteLongReq_t {
    #[doc = "!< Whether reliable writes requested (always FALSE for Write Long)"]
    pub reliable: u8,
    #[doc = "!< ATT Prepare Write Request"]
    pub req: attPrepareWriteReq_t,
    #[doc = "!< Offset of last Prepare Write Request sent"]
    pub lastOffset: u16,
}
#[doc = " GATT Reliable Writes Request format. Do not change the order of the members."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gattReliableWritesReq_t {
    #[doc = "!< Whether reliable writes requested (always TRUE for Reliable Writes)"]
    pub reliable: u8,
    #[doc = "!< Array of Prepare Write Requests (must be allocated)"]
    pub pReqs: *mut attPrepareWriteReq_t,
    #[doc = "!< Number of Prepare Write Requests"]
    pub numReqs: u8,
    #[doc = "!< Index of last Prepare Write Request sent"]
    pub index: u8,
    #[doc = "!< 0x00 - cancel all prepared writes.\n!< 0x01 - immediately write all pending prepared values."]
    pub flags: u8,
}

// dependent types
#[doc = " Attribute Type format (2-octet Bluetooth UUID)."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attAttrBtType_t {
    #[doc = "!< Length of UUID (2)"]
    pub len: u8,
    #[doc = "!< 16 bit UUID"]
    pub uuid: [u8; 2usize],
}

#[doc = " Attribute Type format (2 or 16 octet UUID)."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attAttrType_t {
    #[doc = "!< Length of UUID (2 or 16)"]
    pub len: u8,
    #[doc = "!< 16 or 128 bit UUID"]
    pub uuid: [u8; 16usize],
}

// GATT Attribute Access Permissions Bit Fields
pub const GATT_PERMIT_READ: u8 = 1;
pub const GATT_PERMIT_WRITE: u8 = 2;
pub const GATT_PERMIT_AUTHEN_READ: u8 = 4;
pub const GATT_PERMIT_AUTHEN_WRITE: u8 = 8;
pub const GATT_PERMIT_AUTHOR_READ: u8 = 16;
pub const GATT_PERMIT_AUTHOR_WRITE: u8 = 32;
pub const GATT_PERMIT_ENCRYPT_READ: u8 = 64;
pub const GATT_PERMIT_ENCRYPT_WRITE: u8 = 128;

// GATT local read or write operation
pub const GATT_LOCAL_READ: u32 = 255;
pub const GATT_LOCAL_WRITE: u32 = 254;

// Attribute handle definitions
pub const GATT_INVALID_HANDLE: u32 = 0;
pub const GATT_MIN_HANDLE: u32 = 1;
pub const GATT_MAX_HANDLE: u32 = 65535;

pub const GATT_MAX_MTU: u32 = 65535;

pub const GATT_MAX_NUM_CONN: u32 = 4;

pub const GATT_CFG_NO_OPERATION: u32 = 0;

// GATT Characteristic Properties Bit Fields
pub const GATT_PROP_BCAST: u8 = 1;
pub const GATT_PROP_READ: u8 = 2;
pub const GATT_PROP_WRITE_NO_RSP: u8 = 4;
pub const GATT_PROP_WRITE: u8 = 8;
pub const GATT_PROP_NOTIFY: u8 = 16;
pub const GATT_PROP_INDICATE: u8 = 32;
pub const GATT_PROP_AUTHEN: u8 = 64;
pub const GATT_PROP_EXTENDED: u8 = 128;

// GATT Client Characteristic Configuration Bit Fields
pub const GATT_CLIENT_CFG_NOTIFY: u32 = 1;
pub const GATT_CLIENT_CFG_INDICATE: u32 = 2;

#[doc = " @brief   Callback function prototype to read an attribute value.\n\n @note    blePending can be returned ONLY for the following\n          read operations:\n          - Read Request: ATT_READ_REQ\n          - Read Blob Request: ATT_READ_BLOB_REQ\n\n @note    If blePending is returned then it's the responsibility of the application to respond to\n          ATT_READ_REQ and ATT_READ_BLOB_REQ message with ATT_READ_RSP and ATT_READ_BLOB_RSP\n          message respectively.\n\n @note    Payload 'pValue' used with ATT_READ_RSP and ATT_READ_BLOB_RSP must be allocated using GATT_bm_alloc().\n\n @param   connHandle - connection request was received on\n @param   pAttr - pointer to attribute\n @param   pValue - pointer to data to be read (to be returned)\n @param   pLen - length of data (to be returned)\n @param   offset - offset of the first octet to be read\n @param   maxLen - maximum length of data to be read\n @param   method - type of read message\n\n @return  SUCCESS: Read was successfully.<BR>\n          blePending: A response is pending for this client.<BR>\n          Error, otherwise: ref ATT_ERR_CODE_DEFINES.<BR>"]
pub type pfnGATTReadAttrCB_t =
    Option<
        unsafe extern "C" fn(
            connHandle: u16,
            pAttr: *mut GattAttribute,
            pValue: *mut u8,
            pLen: *mut u16,
            offset: u16,
            maxLen: u16,
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

#[doc = " GATT Attribute Type format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gattAttrType_t {
    #[doc = "!< Length of UUID (2 or 16)"]
    pub len: u8,
    #[doc = "!< Pointer to UUID"]
    pub uuid: *const u8,
}

unsafe impl Sync for gattAttrType_t {}
unsafe impl Send for gattAttrType_t {}

// Renamed from gattAttribute_t
#[doc = " GATT Attribute format."]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct GattAttribute {
    #[doc = "!< Attribute type (2 or 16 octet UUIDs)"]
    pub type_: gattAttrType_t,
    #[doc = "!< Attribute permissions"]
    pub permissions: u8,
    #[doc = "!< Attribute handle - assigned internally by attribute server"]
    pub handle: u16,
    #[doc = "!< Attribute value - encoding of the octet array is defined in\n!< the applicable profile. The maximum length of an attribute\n!< value shall be 512 octets."]
    pub value: *const u8,
}

unsafe impl Sync for GattAttribute {}
unsafe impl Send for GattAttribute {}
