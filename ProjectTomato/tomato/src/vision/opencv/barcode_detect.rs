//Add deref trait in scope
use std::cmp::Ordering;

// use opencv::prelude::*;
use jni::sys::{jboolean, jbyte, jobject};
use jni::JNIEnv;
use opencv::core::{Mat, Point, Rect, Scalar, Size, Vector};
use opencv::types::{VectorOfPoint, VectorOfVectorOfPoint};
use tomato_macros::catch_panic;

use crate::telemetry::get_telemetry_from_pipeline;
use crate::vision::image_provider::from_java_mat;

#[no_mangle]
#[catch_panic]
pub extern "C" fn Java_org_hermitsocialclub_tomato_BarcodeDetect_detect(
    env: JNIEnv,
    _this: jobject,
    mat: jobject,
    pipeline: jobject,
    is_red: jboolean,
) -> jbyte {
    let telemetry = get_telemetry_from_pipeline(env, pipeline);
    //Yoink Java Mat
    let mut og_mat = from_java_mat(env, mat);

    //convert weak RGB to stronk HSV 💪
    let mut hsv = Mat::default();
    opencv::imgproc::cvt_color(&*og_mat, &mut hsv, opencv::imgproc::COLOR_RGB2HSV, 0).unwrap();

    //get the barcode rectangles
    if let Ok(mut barcode) = find_barcode_squares(&hsv, is_red) {
        if let Ok(shipping_element) = find_shipping_element(&hsv) {
            //getting level from ordering of the barcodes
            let shipping_level: i8;
            if shipping_element.x > barcode[0].x {
                if shipping_element.x > barcode[1].x {
                    shipping_level = 3i8
                } else {
                    shipping_level = 2i8
                }
            } else {
                shipping_level = 1i8
            }

            //draw rects
            barcode.push(shipping_element);
            for rect in barcode {
                opencv::imgproc::rectangle(
                    &mut *og_mat,
                    rect,
                    Scalar::new(255.0, 0.0, 0.0, 100.0),
                    5,
                    opencv::imgproc::FILLED,
                    0,
                )
                .unwrap();
            }
            shipping_level
        } else {
            return 4i8;
        }
    } else {
        return 4i8;
    }
}

/**
 * Find the exposed squares of the barcode
 */
fn find_barcode_squares(mat: &Mat, is_red: u8) -> Result<Vec<Rect>, ()> {
    //filter reds and blues
    let mut barcode = Mat::default();
    if is_red != 0 {
        //workaround since red is on both sides of the HSV spectrum 😒
        let mut red_right = Mat::default();
        let lower_target: Vector<i32> = Vector::from_iter([170, 70, 50].into_iter());
        let upper_target: Vector<i32> = Vector::from_iter([180, 255, 255].into_iter());
        opencv::core::in_range(&mat, &lower_target, &upper_target, &mut red_right).unwrap();

        let mut red_left = Mat::default();
        let lower_target: Vector<i32> = Vector::from_iter([0, 70, 50].into_iter());
        let upper_target: Vector<i32> = Vector::from_iter([10, 255, 255].into_iter());
        opencv::core::in_range(&mat, &lower_target, &upper_target, &mut red_left).unwrap();

        opencv::core::bitwise_or(&red_right, &red_left, &mut barcode, &opencv::core::no_array().unwrap()).unwrap();
    } else {
        //blue
        let lower_target: Vector<i32> = Vector::from_iter([100, 50, 100].into_iter());
        let upper_target: Vector<i32> = Vector::from_iter([140, 255, 255].into_iter());
        opencv::core::in_range(&mat, &lower_target, &upper_target, &mut barcode).unwrap();
    }

    //get all square contours
    let mut contours = get_contours(&mut barcode, true);

    //extra panic to make sure stuff works
    if contours.len() < 2 {
        Err(());
    }

    //sort and get the 2 biggest red/blue contours
    contours.sort_by(compare_contour_size);
    let mut biggest_contours: Vec<Rect> = contours
        .into_iter()
        .take(2)
        .map(|contour| opencv::imgproc::bounding_rect(&contour).unwrap())
        .collect();

    //sort from left-to-right
    biggest_contours.sort_by_key(|bounding_box| bounding_box.x);

    Ok(biggest_contours);
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
        Err(());
    }
    let biggest_contour = contours.into_iter().max_by(|a, b| compare_contour_size(b, a)).unwrap();

    Ok(opencv::imgproc::bounding_rect(&biggest_contour).unwrap())
}

/**
 * Utility function that searches (and optionally filters) contours
 */
fn get_contours(mat: &mut Mat, get_squared: bool) -> Vec<VectorOfPoint> {
    //blur image to reduce the chance that a random thing gets picked up as noise
    let mut blurred_mat = Mat::default();
    opencv::imgproc::gaussian_blur(
        mat,
        &mut blurred_mat,
        Size { width: 9, height: 9 },
        0.0,
        0.0,
        opencv::core::BORDER_DEFAULT,
    )
    .unwrap();

    //find all blobs in mask
    let mut contours = VectorOfVectorOfPoint::new();
    opencv::imgproc::find_contours(
        &mut blurred_mat,
        &mut contours,
        opencv::imgproc::RETR_EXTERNAL,
        opencv::imgproc::CHAIN_APPROX_SIMPLE,
        Point::new(0, 0),
    )
    .unwrap();

    //extra method to filter out all non-square contours on scene
    if get_squared {
        // filter out all contours that are not squares
        let contours: Vec<Vector<Point>> = contours
            .iter()
            .map(|a| {
                let mut approx = VectorOfPoint::new();
                let mut hull = VectorOfPoint::new();
                opencv::imgproc::approx_poly_dp(
                    &a,
                    &mut approx,
                    0.02 * opencv::imgproc::arc_length(&a, true).unwrap(),
                    true,
                )
                .unwrap();
                opencv::imgproc::convex_hull(&approx, &mut hull, false, true).unwrap();
                approx
            })
            .filter(|hull| hull.len() < 7)
            .collect();

        return contours;
    }

    contours.to_vec()
}

/**
 * compares contour sizes cuz damn rust is verbose
 * me when floats are not an Ord
 */
fn compare_contour_size(a: &Vector<Point>, b: &Vector<Point>) -> Ordering {
    opencv::imgproc::contour_area(&b, false)
        .unwrap()
        .partial_cmp(&(opencv::imgproc::contour_area(&a, false).unwrap()))
        .unwrap()
}
