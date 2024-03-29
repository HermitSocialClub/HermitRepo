package org.hermitsocialclub.hydra.vision

import org.hermitsocialclub.hydra.vision.util.VisionUtils.toMatOfPoint2f
import org.hermitsocialclub.hydra.vision.util.VisionUtils.zero
import org.opencv.core.*
import org.opencv.imgproc.Imgproc.*

class DistanceToObjectDetector : IVisionPipelineComponent {

    override fun apply(image: Mat, pipeline: VisionPipeline): Mat {
        val gray = zero(image)
        val edged = zero(image)
        cvtColor(image, gray, COLOR_BGR2GRAY)
        GaussianBlur(gray, gray, Size(5.0, 5.0), 0.0)
        Canny(gray, edged, pipeline.cannyLowerThreshold, pipeline.cannyUpperThreshold)

        val cnts: List<MatOfPoint> = ArrayList()
        findContours(edged, cnts, Mat(), RETR_LIST, CHAIN_APPROX_SIMPLE)
        if (cnts.isNotEmpty()) {
            pipeline.telemetry.setData("Contours found", "true")
            // val c = cnts.stream().max { o1: Mat, o2: Mat -> (contourArea(o1) - contourArea(o2)).roundToInt() }
            for (c in cnts) {
                val boundingBox = minAreaRect(toMatOfPoint2f(c))
                val boundingBoxPoints = arrayOfNulls<Point>(4)
                boundingBox.points(boundingBoxPoints)
                var actualPoints =
                    boundingBoxPoints.toList().zipWithNext().map { MatOfPoint(it.first, it.second) }.toMutableList()
                actualPoints.add(MatOfPoint(boundingBoxPoints[boundingBoxPoints.size - 1], boundingBoxPoints[0]))
                polylines(image, actualPoints, true, Scalar(69.0, 230.0, 255.0))
            }
        } else {
            pipeline.telemetry.setData("Contours found", "false")
        }
        return image
    }
}
