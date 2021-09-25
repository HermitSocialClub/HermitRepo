use std::ops::DerefMut;

use jni::sys::jobject;
use jni::JNIEnv;
use opencv::core::{Point, Scalar};

use crate::pipeline::from_java_mat;

#[no_mangle]
pub extern "C" fn Java_org_hermitsocialclub_tomato_NativeTestPipelineComponent_apply(
    env: JNIEnv,
    mat: jobject,
    _pipeline: jobject,
) -> jobject {
    let mut rust_mat = from_java_mat(env, mat);
    opencv::imgproc::put_text(
        rust_mat.deref_mut(),
        "Hello Tomato!",
        Point::new(0, 0),
        opencv::imgproc::FONT_HERSHEY_SIMPLEX,
        1.0,
        Scalar::new(255.0, 48.0, 69.0, 255.0),
        1,
        opencv::imgproc::LINE_8,
        false,
    )
    .unwrap();
    mat
}
