//Add deref trait in scope
use std::ops::Deref;
use opencv::prelude::*;

use jni::sys::{jboolean, jbyte, jobject};
use jni::JNIEnv;
use opencv::core::{Mat, Vector, Point};
use opencv::types::VectorOfVectorOfPoint;


use crate::vision::image_provider::from_java_mat;

#[no_mangle]
pub extern "C" fn Java_org_hermitsocialclub_tomato_BarcodeDetect_detect(
    env: JNIEnv,
    mat: jobject,
    _pipeline: jobject,
    _is_red: jboolean,
) -> jbyte {
    let rust_mat = from_java_mat(env, mat);
    let rust_mat = rust_mat.deref();
    let result: i8 = 1;

    let _padding = 20;
    let _lower_green: Vector<i32> = Vector::from_iter([36, 50, 0].into_iter());
    let _upper_green: Vector<i32> = Vector::from_iter([86, 255, 255].into_iter());

    let mut lower_target: Vector<i32> = Vector::new();
    let mut upper_target: Vector<i32> = Vector::new();

    lower_target.push(155);
    lower_target.push(50);
    lower_target.push(0);

    upper_target.push(179);
    upper_target.push(255);
    upper_target.push(255);

    let mut barcode = Mat::default();

    opencv::core::in_range(rust_mat, &lower_target, &upper_target, &mut barcode).unwrap();

	let mut contours = Mat::default();

	opencv::imgproc::find_contours(
		&barcode,
		&mut contours,
		opencv::imgproc::RETR_EXTERNAL,
		opencv::imgproc::CHAIN_APPROX_SIMPLE,
        Point::new(0, 0)
	).unwrap();



    result
}
