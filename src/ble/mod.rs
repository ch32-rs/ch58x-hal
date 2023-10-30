use core::ffi::CStr;

use crate::{pac, println};

pub mod ffi;

pub fn lib_version() -> &'static str {
    unsafe {
        let version = CStr::from_ptr(ffi::VER_LIB.as_ptr());
        version.to_str().unwrap()
    }
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
