use std::{env, path::PathBuf};

use cbindgen::Config;

fn main() {
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());

    let output_platform_dir = out_path.join("c_inc");
    let output_platform_file = output_platform_dir
        .join("platform.h")
        .display()
        .to_string();

    let config = Config {
        language: cbindgen::Language::C,
        no_includes: true,
        includes: vec!["stdint.h".to_string(), "stdbool.h".to_string()],
        header: Some("#ifndef VL53L5CX_API_H\n#define VL53L5CX_API_H\n".to_string()),
        trailer: Some("#endif\n".to_string()),
        ..Default::default()
    };
    let crate_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    cbindgen::generate_with_config(&crate_dir, config)
        .unwrap()
        .write_to_file(&output_platform_file);

    let config = Config {
        language: cbindgen::Language::C,
        header: Some("#ifndef VL53L5CX_API_H\n#define VL53L5CX_API_H\n".to_string()),
        trailer: Some("#endif\n".to_string()),
        ..Default::default()
    };
    cbindgen::generate_with_config(&crate_dir, config)
        .unwrap()
        .write_to_file(&crate_dir.join("debug.h").display().to_string());

        
    
    //generate rust headers
    let rust_bindings = bindgen::Builder::default()
        .use_core()
        .clang_arg(format!("-I{}", output_platform_dir.display().to_string()))
        .header("c_src/inc/vl53l5cx_api.h")
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate()
        .expect("Unable to generate bindings");

    
    rust_bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");

    let mut build = &mut cc::Build::new();
    let target = std::env::var("TARGET").unwrap();
    if target == "xtensa-esp32s3-none-elf" {
        build = build
            .compiler("xtensa-esp32s3-elf-gcc")
            .flag("-mlongcalls")
            ;
    }

    build
        .include("c_src/inc")
        .include(output_platform_dir.display().to_string())
        .file("c_src/src/vl53l5cx_api.c")
        .compile("vl53l5cx_api");

}