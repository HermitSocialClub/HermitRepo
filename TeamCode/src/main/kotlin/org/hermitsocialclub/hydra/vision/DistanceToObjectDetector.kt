package org.hermitsocialclub.hydra.vision

import org.hermitsocialclub.hydra.vision.VisionUtils.toMatOfPoint2f
import org.hermitsocialclub.telecat.PersistantTelemetry
import org.opencv.core.*
import org.opencv.imgproc.Imgproc.*
import java.util.*
import kotlin.collections.ArrayList
import kotlin.math.roundToInt

class DistanceToObjectDetector : IVisionPipelineComponent {

    override fun apply(image: Mat, telemetry: PersistantTelemetry): Mat {
        val gray = Mat(image.width(), image.height(), image.depth())
        val edged = Mat(image.width(), image.height(), image.depth())
        cvtColor(image, gray, COLOR_BGR2GRAY)
        GaussianBlur(gray, gray, Size(5.0, 5.0), 0.0)
        Canny(gray, edged, 35.0, 125.0)

        val cnts: List<MatOfPoint> = ArrayList()
        findContours(edged, cnts, Mat(), RETR_LIST, CHAIN_APPROX_SIMPLE)
        if (cnts.isNotEmpty()) {
            telemetry.setData("Contours found", "true")
            val c = cnts.stream().max { o1: Mat, o2: Mat -> (contourArea(o1) - contourArea(o2)).roundToInt() }
            if (c.isPresent) {
                telemetry.setData("Largest contour found", "true")
                telemetry.setData("Contour", Arrays.toString(c.get().toArray()))
                val boundingBox = minAreaRect(toMatOfPoint2f(c.get()))

                val boundingBoxPoints = arrayOfNulls<Point>(4)
                boundingBox.points(boundingBoxPoints)
                val boundingBoxMat = MatOfPoint(*boundingBoxPoints)
                fillConvexPoly(image, boundingBoxMat, Scalar(69.0 / 255, 230.0 / 255, 1.0))
            } else {
                telemetry.setData("Largest contour found", "false")
            }
        } else {
            telemetry.setData("Contours found", "false")
        }
        return image
    }

}