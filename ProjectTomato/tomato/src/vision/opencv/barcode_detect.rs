//Add deref trait in scope
use std::cmp::Ordering;

// use opencv::prelude::*;
use jni::sys::{jboolean, jbyte, jobject};
use jni::JNIEnv;
use opencv::core::{Mat, Point, Rect, Scalar, Vector};
use opencv::types::VectorOfVectorOfPoint;

use crate::vision::image_provider::from_java_mat;

#[no_mangle]
pub extern "C" fn Java_org_hermitsocialclub_tomato_BarcodeDetect_detect(
    env: JNIEnv,
    _this: jobject,
    mat: jobject,
    _pipeline: jobject,
    is_red: jboolean,
) -> jbyte {
    //Yoink Java Mat
    let mut og_mat = from_java_mat(env, mat);
    let mut rust_mat = Mat::default();

    // convert weak BGR mat to strong HSV mat
    opencv::imgproc::cvt_color(&*og_mat, &mut rust_mat, opencv::imgproc::COLOR_RGB2HSV, 0).unwrap();

    let result: i8;

    // define some lower and upper bound colors
    let lower_green: Vector<i32> = Vector::from_iter([50, 70, 80].into_iter());
    let upper_green: Vector<i32> = Vector::from_iter([86, 255, 255].into_iter());
    let mut lower_target: Vector<i32>;
    let mut upper_target: Vector<i32>;

    let mut barcode = Mat::default();

    //define barcode sticker colors
    if is_red != 0 {
        lower_target = Vector::from_iter([155, 50, 0].into_iter());

        upper_target = Vector::from_iter([179, 255, 255].into_iter());
    } else {
        let lower_target: Vector<i32> = Vector::from_iter([100, 50, 100].into_iter());
        let upper_target: Vector<i32> = Vector::from_iter([140, 255, 255].into_iter());
    }
    opencv::core::in_range(&rust_mat, &lower_target, &upper_target, &mut barcode).unwrap();

    //save contours
    let mut contours = VectorOfVectorOfPoint::new();
    opencv::imgproc::find_contours(
        &mut barcode,
        &mut contours,
        opencv::imgproc::RETR_EXTERNAL,
        opencv::imgproc::CHAIN_APPROX_SIMPLE,
        Point::new(0, 0),
    )
    .unwrap();

    // Sorts the contours, and returns the biggest three
    let mut contour_areas_sorted = contours.to_vec();

    if contour_areas_sorted.len() < 3 {
        return 4;
    }

    contour_areas_sorted.sort_by(compare_contour_size);

    let biggest_contours = &contour_areas_sorted[0..3];
    // let mut greenmap = Mat::default();

    // convert contours to bounding boxes, and sorting by left-to-right
    let mut b_boxes: Vec<Rect> = biggest_contours
        .iter()
        .map(|contour| opencv::imgproc::bounding_rect(&contour).unwrap())
        .collect();
    b_boxes.sort_by_key(|bounding_box| bounding_box.x);

    // get biggest green box
    let mut contour_areas: Vec<f64> = Vec::new();
    for bounding_box in b_boxes {
        // draw bounding boxes on image
        opencv::imgproc::rectangle(
            &mut *og_mat,
            bounding_box,
            Scalar::new(255.0, 0.0, 0.0, 100.0),
            5,
            opencv::imgproc::FILLED,
            0,
        )
        .unwrap();

        let region_of_interest = Mat::roi(&rust_mat, bounding_box).unwrap();

        // get green spots into mask
        let mut green_mask = Mat::default();
        opencv::core::in_range(&region_of_interest, &lower_green, &upper_green, &mut green_mask).unwrap();

        // find green contours
        let mut contours = VectorOfVectorOfPoint::new();
        opencv::imgproc::find_contours(
            &mut green_mask,
            &mut contours,
            opencv::imgproc::RETR_EXTERNAL,
            opencv::imgproc::CHAIN_APPROX_SIMPLE,
            Point::new(0, 0),
        )
        .unwrap();

        // get biggest contour and add it to the list of green contours
        if contours.is_empty() {
            contour_areas.push(0.0);
        } else {
            let biggest_contour_area: f64 = contours
                .iter()
                .map(|contour| opencv::imgproc::contour_area(&contour, false).unwrap())
                .max_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap();
            contour_areas.push(biggest_contour_area)
        }
    }

    //get square with biggest contour
    result = contour_areas
        .into_iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .map(|(index, _)| index)
        .unwrap() + 1 as i8;

    result
}

//compares contour sizes cuz damn rust is verbose
// me when floats are not an Ord
fn compare_contour_size(a: &Vector<Point>, b: &Vector<Point>) -> Ordering {
    opencv::imgproc::contour_area(&b, false)
        .unwrap()
        .partial_cmp(&(opencv::imgproc::contour_area(&a, false).unwrap()))
        .unwrap()
}
