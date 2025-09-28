use std::{env, path::PathBuf};

use cbindgen::Config;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=src");
    println!("cargo:rerun-if-changed=c_src");

    let crate_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    let target = std::env::var("TARGET").unwrap();
    let is_esp32 = target == "xtensa-esp32s3-espidf";

    // Generate C platform bindings, for all VL53 variants
    // Only generate "raw" bindings. This file will be included to add configuration, system headers, etc.
    let output_platform_dir = out_path.join("c_inc");
    let output_platform_header = output_platform_dir.join("platform_cbindgen.h");
    let config = Config {
        language: cbindgen::Language::C,
        include_guard: Some("VLX_PLATFORM_CBINDGEN".into()),
        no_includes: true,
        sys_includes: vec!["stdint.h".into(), "stdbool.h".into()],
        ..Default::default()
    };
    cbindgen::generate_with_config(&crate_dir, config)
        .unwrap()
        .write_to_file(&output_platform_header);

    let mut build = &mut cc::Build::new();
    if is_esp32 {
        build = build
            .compiler("xtensa-esp32s3-elf-gcc")
            .flag("-mlongcalls")
            ;
    }

    // Build VL53L1X C library
    {
        build
            .include(&output_platform_dir)
            .include("c_src")
            .include("c_src/vl53l1x_platform")
            .file("c_src/vl53l1x_upstream/core/VL53L1X_api.c")
            .compile("vl53l1x_api");
    }

    // Build VL53L5CX C library
    {
        build
            .include(&output_platform_dir)
            .include("c_src")
            .include("c_src/vl53l5cx_platform")
            .include("c_src/vl53l5cx_upstream/VL53L5CX_ULD_API/inc")
            .file("c_src/vl53l5cx_upstream/VL53L5CX_ULD_API/src/vl53l5cx_api.c")
            .file("c_src/vl53l5cx_upstream/VL53L5CX_ULD_API/src/vl53l5cx_plugin_detection_thresholds.c")
            .flag("-Wno-unused-variable")
            .compile("vl53l5cx_api");
    }

    // Generate combined Rust bindings
    bindgen::Builder::default()
        .use_core()
        .clang_arg(format!("-I{}", output_platform_dir.display()))
        .clang_arg(format!("-I{}", crate_dir.join("c_src").display()))
        .clang_arg(format!("-I{}", crate_dir.join("c_src/vl53l1x_platform").display()))
        .clang_arg(format!("-I{}", crate_dir.join("c_src/vl53l5cx_platform").display()))
        .header(crate_dir.join("c_src/combined_bindgen.h").display().to_string())
        .generate()
        .expect("Unable to generate combined bindings")
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}
