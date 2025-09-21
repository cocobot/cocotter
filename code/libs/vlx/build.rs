use std::{env, fs::File, io::Write, path::PathBuf};

use cbindgen::Config;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=src");
    println!("cargo:rerun-if-changed=c_src");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());

    let output_platform_dir = out_path.join("c_inc");
    let output_platform_file = output_platform_dir
        .join("platform.h")
        .display()
        .to_string();
    let output_platform_l1 = output_platform_dir
        .join("vl53l1_platform.h")
        .display()
        .to_string();

    const VL53L5_CONF :[&str; 7] = [
        "#define VL53L5CX_DISABLE_AMBIENT_PER_SPAD\n",
        "#define VL53L5CX_DISABLE_NB_SPADS_ENABLED\n",
        "#define VL53L5CX_DISABLE_NB_TARGET_DETECTED\n",
        "#define VL53L5CX_DISABLE_SIGNAL_PER_SPAD\n",
        "#define VL53L5CX_DISABLE_RANGE_SIGMA_MM\n",
        "#define VL53L5CX_DISABLE_REFLECTANCE_PERCENT\n",
        "#define VL53L5CX_DISABLE_MOTION_INDICATOR\n",
    ];

    let config = Config {
        language: cbindgen::Language::C,
        no_includes: true,
        includes: vec!["stdint.h".to_string(), "stdbool.h".to_string()],
        header: Some("#ifndef VL53L5CX_API_H\n#define VL53L5CX_API_H\n".to_string() + &VL53L5_CONF.join("")),
        trailer: Some("#endif\n".to_string()),
        ..Default::default()
    };
    let crate_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    cbindgen::generate_with_config(&crate_dir, config)
        .unwrap()
        .write_to_file(&output_platform_file);

    let mut file = File::create(output_platform_l1).unwrap();
    file.write_all("#include \"platform.h\"\n".as_bytes()).unwrap();

    // Create a combined header file for both APIs
    let l5_header_path = crate_dir.join("c_src/STSW-IMG023/VL53L5CX_ULD_driver_2.0.1/VL53L5CX_ULD_API/inc/vl53l5cx_api.h");
    let l1_header_path = crate_dir.join("c_src/STSW-IMG009/STSW-IMG009_v3.5.4/API/core/VL53L1X_api.h");
    
    let combined_header_path = output_platform_dir.join("combined_api.h");
    let mut combined_header = File::create(&combined_header_path).unwrap();
    combined_header.write_all(b"#include \"platform.h\"\n").unwrap();
    combined_header.write_all(format!("#include \"{}\"\n", l5_header_path.display()).as_bytes()).unwrap();
    
    // Include VL53L5CX plugins
    let l5_detection_plugin = crate_dir.join("c_src/STSW-IMG023/VL53L5CX_ULD_driver_2.0.1/VL53L5CX_ULD_API/inc/vl53l5cx_plugin_detection_thresholds.h");
    combined_header.write_all(format!("#include \"{}\"\n", l5_detection_plugin.display()).as_bytes()).unwrap();
    
    combined_header.write_all(format!("#include \"{}\"\n", l1_header_path.display()).as_bytes()).unwrap();
    
    // Don't include platform.h as it causes conflicts - just include original headers

    //generate combined rust headers
    let mut bindgen_builder = bindgen::Builder::default()
        .use_core()
        // Include platform dir again since we need minimal platform.h
        .clang_arg(format!("-I{}", output_platform_dir.display().to_string()))
        .clang_arg(format!("-I{}", crate_dir.join("c_src/STSW-IMG023/VL53L5CX_ULD_driver_2.0.1/VL53L5CX_ULD_API/inc").display()))
        .clang_arg(format!("-I{}", crate_dir.join("c_src/STSW-IMG009/STSW-IMG009_v3.5.4/API/core").display()))
        .header(combined_header_path.display().to_string());
    let target = std::env::var("TARGET").unwrap();
    let is_esp32 = target == "xtensa-esp32s3-espidf";
    
    // Configure bindgen for consistent pointer sizes
    if is_esp32 {
        bindgen_builder = bindgen_builder.clang_arg("--target=i686-unknown-linux-gnu");
    } else {
        bindgen_builder = bindgen_builder.clang_arg("--target=x86_64-unknown-linux-gnu");
    }
    
    let rust_bindings = bindgen_builder
        .generate()
        .expect("Unable to generate combined bindings");

    rust_bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
    let mut build = &mut cc::Build::new();
    if is_esp32 {
        build = build
            .compiler("xtensa-esp32s3-elf-gcc")
            .flag("-mlongcalls")
            ;
    }

    build
        .include("c_src/STSW-IMG023/VL53L5CX_ULD_driver_2.0.1/VL53L5CX_ULD_API/inc")
        .include(output_platform_dir.display().to_string())
        .file("c_src/STSW-IMG023/VL53L5CX_ULD_driver_2.0.1/VL53L5CX_ULD_API/src/vl53l5cx_api.c")
        .file("c_src/STSW-IMG023/VL53L5CX_ULD_driver_2.0.1/VL53L5CX_ULD_API/src/vl53l5cx_plugin_detection_thresholds.c")
        .flag("-Wno-unused-variable")
        .compile("vl53l5cx_api");

    build
        .include(output_platform_dir.display().to_string())
        .file("c_src/STSW-IMG009/STSW-IMG009_v3.5.4/API/core/VL53L1X_api.c")
        .compile("vl53l1cx_api");

}