package org.hermitsocialclub.hydra.vision

import org.opencv.core.Mat
import org.opencv.core.MatOfByte
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.imgcodecs.Imgcodecs.*

object VisionUtils {

    fun toMatOfPoint2f(input: MatOfPoint): MatOfPoint2f {
        return MatOfPoint2f(*input.toArray())
    }

    fun loadImage(pipeline: VisionPipeline, name: String, imreadFlags: Int): Mat {
        return pipeline.hardwareMap.appContext.resources.assets.open(name).use {
            imdecode(MatOfByte(*it.readBytes()), imreadFlags)
        }
    }

    fun loadImageColor(pipeline: VisionPipeline, name: String): Mat {
        return loadImage(pipeline, name, IMREAD_COLOR)
    }

    fun loadImageGrayscale(pipeline: VisionPipeline, name: String): Mat {
        return loadImage(pipeline, name, IMREAD_GRAYSCALE)
    }

}