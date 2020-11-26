package org.hermitsocialclub.hydra.vision

import org.hermitsocialclub.hydra.vision.util.VisionUtils.zero
import org.opencv.core.Core.*
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc.*
import kotlin.math.max

/**
 * # StaccDetecc
 * Based on the python implementation [here](https://gist.github.com/Arc-blroth/4e2a66b648b89e4fbc9cc70f78d776b9)
 *
 * @author Arc'blroth
 */
class StaccDetecc(val config: StaccConfig = StaccConfig()) : IVisionPipelineComponent {

    class StaccConfig {
        var normUpper = 300.0
        var lowerYellow = Scalar(19.0, 50.0, 0.0)
        var upperYellow = Scalar(46.0, 255.0, 255.0)
        var maxStackRatio = 1.5
    }

    override fun apply(image: Mat, pipeline: VisionPipeline): Mat {
        // Filter image for a certain color
        val hsvImg = zero(image)
        cvtColor(image, hsvImg, COLOR_RGB2HSV)
        normalize(hsvImg, hsvImg, 0.0, config.normUpper, NORM_MINMAX)
        val colorFilter = zero(hsvImg)
        inRange(hsvImg, config.lowerYellow, config.upperYellow, colorFilter)

        // Find largest square area in image
        try {
            val fsout = findStacc(image, colorFilter)
            if (fsout != null) {
                pipeline.telemetry.setData("Stacc found", "true")
                rectangle(image, fsout, Scalar(0.0, 255.0, 0.0), 3)
            } else {
                pipeline.telemetry.setData("Stacc found", "false")
            }
        } catch (ex: NullPointerException) {
            pipeline.telemetry.setData("Stacc found", "false (NPE)")
        }

        return image
    }

    private fun findStacc(image: Mat, filter: Mat): Rect? {
        filter.convertTo(filter, CvType.CV_8U)
        val labels = Mat()
        val stats = Mat()
        val centroids = Mat()
        val nbComponents = connectedComponentsWithStats(filter, labels, stats, centroids, 4)

        println("Stats")
        println(stats.dump())

        var maxLabel: Int = -1
        var maxSize: Int = -1
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

}