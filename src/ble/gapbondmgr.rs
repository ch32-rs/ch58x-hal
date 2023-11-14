//! GAP Bond Manager



extern "C" {
    #[doc = " @brief       Set a GAP Bond Manager parameter.\n\n @note    You can call this function with a GAP Parameter ID and it will set the GAP Parameter.\n\n @param   param - Profile parameter ID: @ref GAPBOND_PROFILE_PARAMETERS\n @param   len - length of data to write\n @param   pValue - pointer to data to write.  This is dependent on\n          the parameter ID and WILL be cast to the appropriate\n          data type (example: data type of uint16_t will be cast to\n          uint16_t pointer).\n\n @return      SUCCESS or INVALIDPARAMETER (invalid paramID)"]
    pub fn GAPBondMgr_SetParameter(param: u16, len: u8, pValue: *const ::core::ffi::c_void) -> bStatus_t;

    #[doc = " @brief   Get a GAP Bond Manager parameter.\n\n @note    You can call this function with a GAP Parameter ID and it will get a GAP Parameter.\n\n @param   param - Profile parameter ID: @ref GAPBOND_PROFILE_PARAMETERS\n @param   pValue - pointer to location to get the value.  This is dependent on\n          the parameter ID and WILL be cast to the appropriate data type.\n          (example: data type of uint16_t will be cast to uint16_t pointer)\n\n @return      SUCCESS or INVALIDPARAMETER (invalid paramID)"]
    pub fn GAPBondMgr_GetParameter(param: u16, pValue: *mut ::core::ffi::c_void) -> bStatus_t;

    #[doc = " @brief   Respond to a passcode request.\n\n @param   connectionHandle - connection handle of the connected device or 0xFFFF if all devices in database.\n @param   status - SUCCESS if passcode is available, otherwise see @ref SMP_PAIRING_FAILED_DEFINES.\n @param   passcode - integer value containing the passcode.\n\n @return  SUCCESS - bond record found and changed\n          bleIncorrectMode - Link not found."]
    pub fn GAPBondMgr_PasscodeRsp(connectionHandle: u16, status: u8, passcode: u32) -> bStatus_t;

    #[doc = " @brief   Respond to a passcode request.\n\n @param   connHandle - connection handle of the connected device or 0xFFFF if all devices in database.\n @param   status - SUCCESS if oob data is available, otherwise see @ref SMP_PAIRING_FAILED_DEFINES.\n @param   oob - containing the oob data.\n @param   c_peer - containing the peer confirm.\n\n @return  SUCCESS - bond record found and changed\n          bleIncorrectMode - Link not found."]
    pub fn GAPBondMgr_OobRsp(connHandle: u16, status: u8, oob: *mut u8, c_peer: *mut u8) -> bStatus_t;

    #[doc = " @brief   Initialization function for the ecc-function callback.\n\n @param   pEcc - callback registration Structure @ref gapEccCBs_t.\n\n @return  null."]
    pub fn GAPBondMgr_EccInit(pEcc: *mut gapEccCBs_t);

    #[doc = " @brief   Send a security request\n\n @param   connHandle - connection handle\n\n @return  SUCCESS: will send\n          bleNotConnected: Link not found\n          bleIncorrectMode: wrong GAP role, must be a Peripheral Role"]
    pub fn GAPBondMgr_PeriSecurityReq(connHandle: u16) -> bStatus_t;
}
