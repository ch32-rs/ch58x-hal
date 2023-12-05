#![allow(
    non_camel_case_types,
    non_snake_case,
    non_upper_case_globals,
    unused_variables,
    non_camel_case_types
)]

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct linkSec_t {
    #[doc = "!< Signature Resolving Key"]
    pub srk: [u8; 16usize],
    #[doc = "!< Sign Counter"]
    pub signCounter: u32,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct encParams_t {
    #[doc = "!< Long Term Key"]
    pub ltk: [u8; 16usize],
    #[doc = "!< Diversifier"]
    pub div: u16,
    #[doc = "!< random number"]
    pub rand: [u8; 8usize],
    #[doc = "!< LTK Key Size"]
    pub keySize: u8,
    pub gapBondInvalid: u8,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct linkDBItem_t {
    #[doc = "!< Application that controls the link"]
    pub taskID: u8,
    #[doc = "!< Controller connection handle"]
    pub connectionHandle: u16,
    #[doc = "!< LINK_CONNECTED, LINK_AUTHENTICATED..."]
    pub stateFlags: u8,
    #[doc = "!< Address type of connected device"]
    pub addrType: u8,
    #[doc = "!< Other Device's address"]
    pub addr: [u8; 6usize],
    #[doc = "!< Connection formed as central or peripheral"]
    pub connRole: u8,
    #[doc = "!< The connection's interval (n * 1.25ms)"]
    pub connInterval: u16,
    pub connLatency: u16,
    pub connTimeout: u16,
    #[doc = "!< The connection's MTU size"]
    pub MTU: u16,
    #[doc = "!< Connection Security related items"]
    pub sec: linkSec_t,
    #[doc = "!< pointer to LTK, ediv, rand. if needed."]
    pub pEncParams: *mut encParams_t,
    pub smEvtID: u16,
    pub pPairingParams: *mut ::core::ffi::c_void,
    pub pAuthLink: *mut ::core::ffi::c_void,
}

pub type pfnLinkDBCB_t = ::core::option::Option<unsafe extern "C" fn(connectionHandle: u16, changeType: u8)>;

pub type pfnPerformFuncCB_t = ::core::option::Option<unsafe extern "C" fn(pLinkItem: *mut linkDBItem_t)>;

extern "C" {
    /// Register with this function to receive a callback when
    /// status changes on a connection.
    pub fn linkDB_Register(pFunc: pfnLinkDBCB_t) -> u8;
    /// Check to see if a physical link is in a specific state.
    ///
    /// returns TRUE is the link is in state. FALSE, otherwise.
    pub fn linkDB_State(connectionHandle: u16, state: u8) -> u8;

    /// Perform a function of each connection in the link database.
    pub fn linkDB_PerformFunc(cb: pfnPerformFuncCB_t);
}

// Link state flags
pub const LINK_NOT_CONNECTED: u8 = 0;
pub const LINK_CONNECTED: u8 = 1;
pub const LINK_AUTHENTICATED: u8 = 2;
pub const LINK_BOUND: u8 = 4;
pub const LINK_ENCRYPTED: u8 = 16;

// Link Database Status callback changeTypes
pub const LINKDB_STATUS_UPDATE_NEW: u32 = 0;
pub const LINKDB_STATUS_UPDATE_REMOVED: u32 = 1;
pub const LINKDB_STATUS_UPDATE_STATEFLAGS: u32 = 2;

pub struct LinkDB;

impl LinkDB {
    pub fn register(pFunc: pfnLinkDBCB_t) -> u8 {
        unsafe { linkDB_Register(pFunc) }
    }

    pub fn state(connectionHandle: u16, state: u8) -> u8 {
        unsafe { linkDB_State(connectionHandle, state) }
    }

    pub fn perform_func(cb: pfnPerformFuncCB_t) {
        unsafe { linkDB_PerformFunc(cb) }
    }

    pub fn is_connected(connectionHandle: u16) -> bool {
        Self::state(connectionHandle, LINK_CONNECTED) != 0
    }
}
