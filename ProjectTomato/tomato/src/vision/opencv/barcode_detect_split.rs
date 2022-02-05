// use opencv::prelude::*;
use jni::sys::{jboolean, jbyte, jobject};
use jni::JNIEnv;
use opencv::core::{Mat, Point, Rect, Scalar, Size, Vector};
use opencv::types::{VectorOfPoint, VectorOfVectorOfPoint};
use tomato_macros::catch_panic;

use crate::telemetry::get_telemetry_from_pipeline;
use crate::vision::image_provider::from_java_mat;
use crate::vision::opencv::barcode_detect::{compare_contour_size, get_contours};

#[no_mangle]
#[catch_panic]
pub extern "C" fn Java_org_hermitsocialclub_tomato_BarcodeDetect_detect_split(
    env: JNIEnv,
    _this: jobject,
    mat: jobject,
    pipeline: jobject,
    is_red: jboolean,
) -> jbyte {
    let mut og_mat = from_java_mat(env, mat);
    //convert weak RGB to stronk HSV 💪
    let mut hsv = Mat::default();
    opencv::imgproc::cvt_color(&*og_mat, &mut hsv, opencv::imgproc::COLOR_RGB2HSV, 0).unwrap();

    if let Ok(shipping_element) = find_shipping_element(&hsv) {
        opencv::imgproc::rectangle(
            &mut *og_mat,
            shipping_element,
            Scalar::new(255.0, 0.0, 0.0, 100.0),
            5,
            opencv::imgproc::FILLED,
            0,
        )
        .unwrap();
    }
    return 4i8;
}

/**
 * A method to find the shipping element in the scene
 */
fn find_shipping_element(mat: &Mat) -> Result<Rect, ()> {
    //green filter
    let mut shipping_element = Mat::default();
    let lower_green: Vector<i32> = Vector::from_iter([40, 100, 0].into_iter());
    let upper_green: Vector<i32> = Vector::from_iter([86, 255, 255].into_iter());
    opencv::core::in_range(&mat, &lower_green, &upper_green, &mut shipping_element).unwrap();

    //get green blob
    let contours = get_contours(&mut shipping_element, false);

    if contours.len() < 1 {
        // panic!("cannot detect green shipping element");
        return Err(());
    }
    let biggest_contour = contours.into_iter().max_by(|a, b| compare_contour_size(b, a)).unwrap();

    Ok(opencv::imgproc::bounding_rect(&biggest_contour).unwrap())
}
