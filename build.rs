use std::path::PathBuf;
use std::{env, fs};

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    fs::write(out_dir.join("libISP583.a"), include_bytes!("vendor/libISP583.a")).unwrap();

    fs::write(out_dir.join("libCH58xBLE.a"), include_bytes!("vendor/LIBCH58xBLE.a")).unwrap();

    // Put the linker script somewhere the linker can find it.
    fs::write(out_dir.join("link.x"), include_bytes!("link.x")).unwrap();

    //fs::write(out_dir.join("memory.x"), include_bytes!("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out_dir.display());

    println!("cargo:rustc-link-search={}", out_dir.display());

    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}
