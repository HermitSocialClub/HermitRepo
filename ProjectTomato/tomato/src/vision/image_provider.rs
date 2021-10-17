use std::ffi::c_void;
use std::mem::ManuallyDrop;

use jni::objects::JValue;
use jni::signature::{JavaType, Primitive};
use jni::sys::jobject;
use jni::JNIEnv;
use opencv::prelude::{Boxed, Mat};

const MAT_CLASS: &str = "org/opencv/core/Mat";

pub unsafe fn get_java_mat_ptr(env: JNIEnv, obj: jobject) -> *mut c_void {
    let field_id = env.get_field_id(MAT_CLASS, "nativeObj", "J").unwrap();
    env.get_field_unchecked(obj, field_id, JavaType::Primitive(Primitive::Long))
        .unwrap()
        .j()
        .unwrap() as *mut c_void
}

/// Converts a Java Mat to a Rust [`Mat`].
///
/// Since the native Mat that is passed to this function is still
/// owned by the Java object, we must ensure that the Rust `Mat`
/// is never dropped, or else we might have double-frees. Thus,
/// the returned `Mat` is wrapped in a `ManuallyDrop`.
/// **The caller must ensure that the returned `Mat` is never
/// dropped**: do not `take()` the Mat out of the `ManuallyDrop`
/// or call `drop()` on the `ManuallyDrop`.
pub fn from_java_mat(env: JNIEnv, obj: jobject) -> ManuallyDrop<Mat> {
    unsafe { ManuallyDrop::new(Mat::from_raw(get_java_mat_ptr(env, obj))) }
}

/// Converts a Rust [`Mat`] to a Java Mat.
///
/// Since the returned Java object must own the Mat,
/// we must ensure that the passed Rust Mat is never
/// dropped.
///
/// **Note**: This method is only intended if you need
/// to return a Mat created in native code. If you only
/// ever modify the Mat passed from Java in-place, return
/// the original `jobject` instead of one created by this
/// function.
pub fn to_java_mat(env: JNIEnv, mat: Mat) -> jobject {
    let mat_class = env.find_class(MAT_CLASS).unwrap();
    let mat_ctor = env.get_method_id(mat_class, "<init>", "(J)V").unwrap();
    let mat_ptr = mat.into_raw() as i64;
    env.new_object_unchecked(mat_class, mat_ctor, &[JValue::Long(mat_ptr)])
        .unwrap()
        .into_inner()
}
