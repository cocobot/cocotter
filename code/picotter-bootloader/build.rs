fn main() {
    let out = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    println!("cargo:rustc-link-search={}", out);
    println!("cargo:rerun-if-changed=memory.x");
}
