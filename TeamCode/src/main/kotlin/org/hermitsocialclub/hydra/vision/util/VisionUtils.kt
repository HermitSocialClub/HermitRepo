package org.hermitsocialclub.hydra.vision.util

import org.hermitsocialclub.hydra.vision.VisionPipeline
import org.opencv.core.*
import org.opencv.imgcodecs.Imgcodecs.*
import java.io.File
import java.io.FileInputStream
import java.io.FileOutputStream
import java.util.*

object VisionUtils {

    val EMPTY_MAT = Mat()

    inline fun zero(input: Mat): Mat {
        return Mat(input.width(), input.height(), input.depth())
    }

    inline fun toMatOfPoint2f(input: MatOfPoint): MatOfPoint2f {
        return MatOfPoint2f(*input.toArray())
    }

    fun loadImage(pipeline: VisionPipeline, name: String, imreadFlags: Int): Mat {
        return pipeline.hardwareMap.appContext.resources.assets.open(name).use {
            imdecode(MatOfByte(*it.readBytes()), imreadFlags)
        }
    }

    inline fun loadImageColor(pipeline: VisionPipeline, name: String): Mat {
        return loadImage(pipeline, name, IMREAD_COLOR)
    }

    inline fun loadImageGrayscale(pipeline: VisionPipeline, name: String): Mat {
        return loadImage(pipeline, name, IMREAD_GRAYSCALE)
    }

    @JvmStatic
    fun loadRectFromFile(file: File): Rect {
        val properties = Properties()
        if(!file.exists()) file.createNewFile()
        FileInputStream(file).use {
            properties.load(it)
        }
        return Rect(
            Integer.valueOf(properties.getProperty("x", "0")),
            Integer.valueOf(properties.getProperty("y", "0")),
            Integer.valueOf(properties.getProperty("width", "100")),
            Integer.valueOf(properties.getProperty("height", "100")),
        )
    }

    @JvmStatic
    fun saveRectToFile(rect: Rect, file: File) {
        val properties = Properties()
        properties.setProperty("x", rect.x.toString())
        properties.setProperty("y", rect.y.toString())
        properties.setProperty("width", rect.width.toString())
        properties.setProperty("height", rect.height.toString())
        if(!file.exists()) file.createNewFile()
        FileOutputStream(file).use {
            properties.store(it, null)
        }
    }

}