// use opencv::prelude::*;
use catch_panic::catch_panic;
use jni::sys::{jboolean, jbyte, jobject};
use jni::JNIEnv;
use opencv::core::{Mat, Point, Rect, Scalar, Size, Vector};
use opencv::types::{VectorOfPoint, VectorOfVectorOfPoint};

use crate::telemetry::{get_telemetry_from_pipeline, PersistantTelemetry};
use crate::util::catch_panic_handler;
use crate::vision::image_provider::from_java_mat;
use crate::vision::opencv::barcode_detect::{compare_contour_size, get_contours};

#[no_mangle]
#[catch_panic(handler = "catch_panic_handler")]
pub extern "C" fn Java_org_hermitsocialclub_tomato_DuckDetect_duckDetector(
    env: JNIEnv,
    _this: jobject,
    mat: jobject,
    pipeline: jobject,
) -> jbyte {
    let mut og_mat = from_java_mat(env, mat);
    //convert weak RGB to stronk HSV 💪
    let mut hsv = Mat::default();
    opencv::imgproc::cvt_color(&*og_mat, &mut hsv, opencv::imgproc::COLOR_RGB2HSV, 0).unwrap();
    let width = 320;
    let pt = get_telemetry_from_pipeline(env, pipeline);

    if let Ok(duck) = find_duck(&hsv, &pt) {
        if duck.x > (2 * width / 5) {
            if duck.x < (3 * width / 5) {
                return 2i8;
            } else {
                return 3i8;
            }
        } else {
            return 1i8;
        }
    }
    return 4i8;
}

/**
 * A method to find the duck in the scene
 */

fn find_duck(mat: &Mat, pt: &PersistantTelemetry) -> Result<Rect, ()> {
    //yellow filter
    let mut shipping_element = Mat::default();
    let lower_yellow: Vector<i32> = Vector::from_iter([22, 93, 0].into_iter());
    let upper_yellow: Vector<i32> = Vector::from_iter([45, 255, 255].into_iter());
    opencv::core::in_range(&mat, &lower_yellow, &upper_yellow, &mut shipping_element).unwrap();

    //get yellow blob
    let contours = get_contours(&mut shipping_element, false);

    if contours.len() < 1 {
        // panic!("cannot detect yellow shipping element");
        return Err(());
    }
    let biggest_contour = contours.into_iter().max_by(|a, b| compare_contour_size(b, a)).unwrap();

    let size = opencv::imgproc::contour_area(&biggest_contour, false)
        .unwrap()
        .to_string();
    pt.set_data("sizeThing", size);

    Ok(opencv::imgproc::bounding_rect(&biggest_contour).unwrap())
}
