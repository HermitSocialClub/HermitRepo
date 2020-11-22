package org.hermitsocialclub.hydra.vision

import org.hermitsocialclub.hydra.vision.util.VisionUtils
import org.hermitsocialclub.hydra.vision.util.VisionUtils.EMPTY_MAT
import org.opencv.core.Core.NORM_HAMMING
import org.opencv.core.Mat
import org.opencv.core.MatOfDMatch
import org.opencv.core.MatOfKeyPoint
import org.opencv.core.Scalar
import org.opencv.features2d.BFMatcher
import org.opencv.features2d.Features2d.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
import org.opencv.features2d.Features2d.drawMatchesKnn
import org.opencv.features2d.ORB

class BetterVuforia : IVisionPipelineComponent {

    private lateinit var skystoneImg: Mat
    private lateinit var detector: ORB
    private lateinit var kp1: MatOfKeyPoint
    private lateinit var des1: Mat

    override fun init(visionPipeline: VisionPipeline) {
        skystoneImg = VisionUtils.loadImageGrayscale(visionPipeline, "vision/Skystone.jpg")

        // Precompute keypoints
        kp1 = MatOfKeyPoint()
        des1 = Mat()

        detector = ORB.create()
        detector.detectAndCompute(skystoneImg, EMPTY_MAT, kp1, des1)
    }

    override fun apply(t: Mat, u: VisionPipeline): Mat {
        val kp2 = MatOfKeyPoint()
        val des2 = Mat()
        detector.detectAndCompute(t, EMPTY_MAT, kp2, des2)

        val matcher = BFMatcher(NORM_HAMMING, true)
        val matches = ArrayList<MatOfDMatch>()
        matcher.knnMatch(des1, des2, matches, 2)

        matches.removeIf {
            val array = it.toArray()
            array[0].distance > 0.75 * array[1].distance
        }

        val out = Mat()
        drawMatchesKnn(
            skystoneImg,
            kp1,
            t,
            kp2,
            matches,
            out,
            Scalar.all(-1.0),
            Scalar.all(-1.0),
            emptyList(),
            DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
        )

        return out
    }

}
