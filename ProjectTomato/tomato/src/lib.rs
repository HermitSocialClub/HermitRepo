//! The main vision code for Project Tomato.
//!
//! Most of these top-level methods are exported
//! and invoked through JNI.

pub mod telemetry;
pub mod util;
pub mod vision;

use jni::sys::jobject;
use jni::JNIEnv;
use opencv::prelude::MatTraitConst;
use tomato_macros::catch_panic;

use crate::vision::image_provider::{from_java_mat, get_java_mat_ptr};

#[no_mangle]
pub extern "C" fn Java_org_hermitsocialclub_tomato_LibTomato_splat(_env: JNIEnv) -> i32 {
    69
}

#[no_mangle]
#[catch_panic]
pub extern "C" fn Java_org_hermitsocialclub_tomato_LibTomato_panicTest(_testing: JNIEnv) {
    panic!("Hopefully this doesn't crash everything");
}

/// Deconstructs a Java Mat into a direct Java ByteBuffer,
/// so that we don't need to copy the Mat through Java just
/// for Tensorflow.
///
/// **The returned ByteBuffer is only valid for the lifetime
/// of the given Mat.**
#[no_mangle]
#[catch_panic(default = "std::ptr::null_mut()")]
pub extern "C" fn Java_org_hermitsocialclub_hydra_vision_util_VisionUtils_matAsByteBuffer(
    env: JNIEnv,
    mat: jobject,
) -> jobject {
    let mat_ptr = unsafe { get_java_mat_ptr(env, mat) } as *mut u8;
    let mat = from_java_mat(env, mat);
    if !mat.is_continuous().unwrap_or(false) {
        panic!("matAsByteBuffer only supports continuous Mats");
    }
    let mat_len = mat.total().unwrap() * mat.elem_size().unwrap();
    let mat_slice = unsafe { std::slice::from_raw_parts_mut(mat_ptr, mat_len) };
    env.new_direct_byte_buffer(mat_slice).unwrap().into_inner()
}
