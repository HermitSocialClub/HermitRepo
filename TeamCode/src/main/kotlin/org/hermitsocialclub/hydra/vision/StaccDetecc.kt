package org.hermitsocialclub.hydra.vision

import org.hermitsocialclub.hydra.vision.util.CameraConfig
import org.hermitsocialclub.hydra.vision.util.VisionUtils.zero
import org.hermitsocialclub.telecat.PersistantTelemetry
import org.opencv.calib3d.Calib3d.projectPoints
import org.opencv.calib3d.Calib3d.solvePnP
import org.opencv.core.*
import org.opencv.core.Core.*
import org.opencv.imgproc.Imgproc.*
import kotlin.math.max

/**
 * # StaccDetecc
 * Based on the python implementation [here](https://gist.github.com/Arc-blroth/b96b13f152c77a46746cc9638bb5bd2c)
 *
 * @author Arc'blroth
 */
class StaccDetecc @JvmOverloads constructor(var config: StaccConfig = StaccConfig(), val cameraConfig: CameraConfig? = null) : IVisionPipelineComponent {

    class StaccConfig {
        /**
         * Subregion of the image to look at for stacks.
         */
        var submat: Rect? = null

        /**
         * 300 for extremely bright light conditions and 1000 for extremely dark light conditions.
         */
        var normUpper = 450.0

        /**
         * "Lower" HSV color to look for.
         */
        var lowerYellow = Scalar(0.0, 135.0, 130.0)

        /**
         * "Upper" HSV color to look for.
         */
        var upperYellow = Scalar(255.0, 255.0, 510.0)

        /**
         * Threshold value to separate the top of the ring from the bottom.
         */
        var topThreshold = 127.0

        /**
         * Pixel regions with an area smaller than this are discarded by the algorithm.
         */
        var minStackArea = 512

        /**
         * Pixel regions with a ratio greater than this are discarded by the algorithm.
         */
        var maxStackRatio = Int.MAX_VALUE

        /**
         * If more than this number of pixel regions are detected, we assume there is
         * no stack in the image and that the algorithm is probably picking up on the floor.
         */
        var maxStackNoise = 100

        /**
         * Stacks with a ratio less than this are considered one-ring stacks.
         * Stacks with a ratio greater than this are considered four-ring stacks.
         */
        var oneStackRatio = 0.4
    }

    /**
     * The last stack height we detected. Either 0, 1, or 4.
     */
    var lastStackHeight = 0
        private set

    override fun apply(image: Mat, pipeline: VisionPipeline): Mat {
        val subImage = if (config.submat != null) {
            Mat(image, config.submat)
        } else {
            image
        }

        // Filter image for a certain color
        val hsvImg = zero(subImage)
        cvtColor(subImage, hsvImg, COLOR_RGB2HSV)
        normalize(hsvImg, hsvImg, 0.0, config.normUpper, NORM_MINMAX)
        val colorFilter = zero(hsvImg)
        inRange(hsvImg, config.lowerYellow, config.upperYellow, colorFilter)

        // Find largest square area in image
        try {
            val stackArea = findStacc(colorFilter)
            if (stackArea != null) {
                val stackTopArea = findStaccTop(subImage, colorFilter, stackArea)
                val ratio = (stackArea.height - stackTopArea.height).toDouble() / stackArea.width.toDouble()
                this.lastStackHeight = if (ratio < config.oneStackRatio) 1 else 4
                pipeline.telemetry.setData("Stacc ratio", ratio)
                pipeline.telemetry.setData("Stacc found", "true [$lastStackHeight]")
                pipeline.telemetry.setData(
                    "Ring Area",
                    "[${stackArea.x}, ${stackArea.y}" +
                        ", ${stackArea.width}, ${stackArea.height}]"
                )
                pipeline.telemetry.setData("Top of ring", "[${stackTopArea.x}, ${stackTopArea.y}, ${stackTopArea.width}, ${stackTopArea.height}]")
                // rectangle(subImage, stackArea, Scalar(0.0, 255.0, 0.0), 2)
                rectangle(subImage, stackTopArea, Scalar(0.0, 127.0, 127.0), 2)

                findStaccPose(subImage, stackArea, stackTopArea, pipeline.telemetry)
            } else {
                this.lastStackHeight = 0
                pipeline.telemetry.removeData("Stacc ratio")
                pipeline.telemetry.setData("Stacc found", "false")
            }
        } catch (ex: NullPointerException) {
            this.lastStackHeight = 0
            pipeline.telemetry.removeData("Stacc ratio")
            pipeline.telemetry.setData("Stacc found", "false (NPE)")
        }

        return image
    }

    private fun findStacc(filter: Mat): Rect? {
        filter.convertTo(filter, CvType.CV_8U)
        val labels = Mat()
        val stats = Mat()
        val centroids = Mat()
        val nbComponents = connectedComponentsWithStats(filter, labels, stats, centroids, 4)

        if (nbComponents > config.maxStackNoise) {
            return null
        }

        var maxLabel: Int = -1
        var maxSize: Int = config.minStackArea
        val areaBuffer = intArrayOf(1)
        val widthBuffer = intArrayOf(1)
        val heightBuffer = intArrayOf(1)
        for (i in 2..nbComponents) {
            stats[i, CC_STAT_AREA, areaBuffer]
            if (areaBuffer[0] > maxSize) {
                stats[i, CC_STAT_WIDTH, widthBuffer]
                stats[i, CC_STAT_HEIGHT, heightBuffer]
                if (max(widthBuffer[0] / heightBuffer[0], heightBuffer[0] / widthBuffer[0]) < config.maxStackRatio) {
                    maxLabel = i
                    maxSize = areaBuffer[0]
                }
            }
        }

        return if (maxLabel == -1) {
            null
        } else {
            val leftBuffer = intArrayOf(1)
            val topBuffer = intArrayOf(1)
            stats[maxLabel, CC_STAT_LEFT, leftBuffer]
            stats[maxLabel, CC_STAT_TOP, topBuffer]
            stats[maxLabel, CC_STAT_WIDTH, widthBuffer]
            stats[maxLabel, CC_STAT_HEIGHT, heightBuffer]
            Rect(
                leftBuffer[0],
                topBuffer[0],
                widthBuffer[0],
                heightBuffer[0],
            )
        }
    }

    private fun findStaccTop(image: Mat, colorFilter: Mat, staccArea: Rect): Rect {
        // Find the top of the ring
        val topImage = run {
            val ring = Mat()
            val topFilter = Mat(colorFilter, staccArea)
            val topFilterInv = Mat()
            val top = Mat()
            val topGray = Mat()
            val topGrayMasked = Mat()
            Mat(image, staccArea).copyTo(ring)
            bitwise_not(topFilter, topFilterInv)
            cvtColor(ring, top, COLOR_BGR2GRAY)
            threshold(top, topGray, config.topThreshold, 255.0, THRESH_BINARY)
            bitwise_and(topGray, topGray, topGrayMasked, topFilterInv)
            topGrayMasked
        }

        val labels = Mat()
        val stats = Mat()
        val centroids = Mat()
        val nbComponents = connectedComponentsWithStats(topImage, labels, stats, centroids, 4)

        var maxLabel: Int = -1
        var maxSize = 0
        val areaBuffer = intArrayOf(1)
        val widthBuffer = intArrayOf(1)
        val heightBuffer = intArrayOf(1)
        for (i in 1..nbComponents) {
            stats[i, CC_STAT_AREA, areaBuffer]
            if (areaBuffer[0] > maxSize) {
                stats[i, CC_STAT_WIDTH, widthBuffer]
                stats[i, CC_STAT_HEIGHT, heightBuffer]
                if (max(widthBuffer[0] / heightBuffer[0], heightBuffer[0] / widthBuffer[0]) < config.maxStackRatio) {
                    maxLabel = i
                    maxSize = areaBuffer[0]
                }
            }
        }

        val leftBuffer = intArrayOf(1)
        val topBuffer = intArrayOf(1)
        stats[maxLabel, CC_STAT_LEFT, leftBuffer]
        stats[maxLabel, CC_STAT_TOP, topBuffer]
        stats[maxLabel, CC_STAT_WIDTH, widthBuffer]
        stats[maxLabel, CC_STAT_HEIGHT, heightBuffer]

        return Rect(
            staccArea.x + leftBuffer[0],
            staccArea.y + topBuffer[0],
            widthBuffer[0],
            heightBuffer[0],
        )
    }

    private fun findStaccPose(image: Mat, staccArea: Rect, topArea: Rect, telemetry: PersistantTelemetry) {
        if (cameraConfig == null) {
            return
        }

        val objectPoints = MatOfPoint3f(
            Point3(0.0, 0.0, 0.0),
            Point3(5.0, 0.0, 0.0),
            Point3(0.0, 0.0, 5.0),
            Point3(5.0, 0.0, 5.0),
            Point3(0.0, 0.75, 5.0),
            Point3(5.0, 0.75, 5.0),
        )

        val (corner, corners) = run {
            val left = staccArea.x.toDouble()
            val right = (staccArea.x + staccArea.width).toDouble()
            val top = staccArea.y.toDouble()
            val bottom = (staccArea.y + staccArea.height).toDouble()
            val topBottom = (topArea.y + topArea.height).toDouble()
            Pair(
                Point(left, bottom),
                MatOfPoint2f(
                    Point(left, bottom),
                    Point(right, bottom),
                    Point(left, topBottom),
                    Point(right, topBottom),
                    Point(left, top),
                    Point(right, top),
                )
            )
        }

        // Find pose
        val rvec = Mat()
        val tvec = Mat()
        solvePnP(
            objectPoints,
            corners,
            cameraConfig.cameraMatrix,
            cameraConfig.distortionCoefficients,
            rvec,
            tvec,
            false,
        )

        telemetry.setData("tvec", tvec.dump())

        // Draw pose
        val imgpts = MatOfPoint2f()
        val jac = Mat()
        projectPoints(
            MatOfPoint3f(
                Point3(5.0, 0.0, 0.0),
                Point3(0.0, 5.0, 0.0),
                Point3(0.0, 0.0, 25.0 / 0.75),
            ),
            rvec,
            tvec,
            cameraConfig.cameraMatrix,
            cameraConfig.distortionCoefficients,
            imgpts,
            jac,
        )

        val imgptsArray = imgpts.toArray()
        line(image, corner, imgptsArray[0], Scalar(255.0, 0.0, 0.0), 3)
        line(image, corner, imgptsArray[1], Scalar(0.0, 255.0, 0.0), 3)
        line(image, corner, imgptsArray[2], Scalar(0.0, 0.0, 255.0), 3)
    }
}
