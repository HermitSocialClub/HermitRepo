//! The main vision code for Project Tomato.
//!
//! Most of these top-level methods are exported
//! and invoked through JNI.

pub mod vision;

use jni::JNIEnv;

#[no_mangle]
pub extern "C" fn Java_org_hermitsocialclub_tomato_LibTomato_splat(_env: JNIEnv) -> i32 {
    69
}

#[no_mangle]
pub extern "C" fn placebo_to_force_tflite_linkage() {
    use tflite::op_resolver::OpResolver;
    use tflite::ops::builtin::BuiltinOpResolver;
    println!("{:?}", BuiltinOpResolver::default().get_resolver_handle());
}
