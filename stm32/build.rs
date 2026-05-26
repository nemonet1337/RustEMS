fn main() {
    // Make memory.x available to the linker at the crate root.
    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    println!("cargo:rustc-link-search={manifest}");
    println!("cargo:rerun-if-changed=memory.x");
}
