//! The main vision code for Project Tomato.
//!
//! Most of these top-level methods are exported
//! and invoked through JNI.

pub mod pipeline;

use jni::JNIEnv;

#[no_mangle]
pub extern "C" fn Java_org_hermitsocialclub_tomato_LibTomato_splat(_env: JNIEnv) -> i32 {
    69
}
