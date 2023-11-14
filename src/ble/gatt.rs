#![allow(non_snake_case, non_camel_case_types)]

use super::ffi::tmos_event_hdr_t;

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
