//! The main vision code for Project Tomato.
//!
//! Most of these top-level methods are exported
//! and invoked through JNI.

pub mod util;
pub mod vision;

use jni::JNIEnv;
use tomato_macros::catch_panic;

#[no_mangle]
pub extern "C" fn Java_org_hermitsocialclub_tomato_LibTomato_splat(_env: JNIEnv) -> i32 {
    69
}

#[no_mangle]
#[catch_panic]
pub extern "C" fn Java_org_hermitsocialclub_tomato_LibTomato_panicTest(_testing: JNIEnv) {
    panic!("Hopefully this doesn't crash everything");
}
