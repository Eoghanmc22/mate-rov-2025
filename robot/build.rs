fn main() {
    println!("cargo::rustc-check-cfg=cfg(rpi)");

    let arch = std::env::var("CARGO_CFG_TARGET_ARCH");
    if let Ok(arch) = arch {
        if arch == *"aarch64" || arch.contains("arm") {
            println!("cargo:rustc-cfg=rpi");
        }
    }
}
