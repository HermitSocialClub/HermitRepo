use std::path::PathBuf;

fn main() {
    // Rerun this script if the wrappers are updated
    println!("cargo:rerun-if-changed=wrapper.h");
    println!("cargo:rerun-if-changed=wrapper.cpp");

    // Compile our helper function to steal the pipeline
    // from a T265Camera's `mNativeCameraObjectPointer`.
    cc::Build::new()
        .cpp(true)
        .file("wrapper.cpp")
        .include("../target/ftc265/")
        .compile("ftc265-sys-helper");

    // Generate bindings
    let bindings = bindgen::Builder::default()
        .header("wrapper.h")
        .parse_callbacks(Box::new(bindgen::CargoCallbacks))
        .size_t_is_usize(true)
        .generate()
        .expect("Unable to generate librealsense bindings!");

    let out_path = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}
