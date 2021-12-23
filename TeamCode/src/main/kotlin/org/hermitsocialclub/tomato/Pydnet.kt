package org.hermitsocialclub.tomato

import android.os.Environment
import org.hermitsocialclub.hydra.vision.IVisionPipelineComponent
import org.hermitsocialclub.hydra.vision.VisionPipeline
import org.hermitsocialclub.telecat.PersistantTelemetry
import org.opencv.core.Core
import org.opencv.core.Core.rotate
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc
import org.opencv.imgproc.Imgproc.resize
import unibo.cvlab.pydnet.PydnetPP
import unibo.cvlab.pydnet.Utils
import java.io.File

/**
 * ## Mobile Pydnet v1
 *
 * A wrapper around the
 * [Pydnet TFLite model](https://github.com/FilippoAleotti/mobilePydnet/tree/ddd472fca53c869f81273cb187bbad5a205ba883)
 * which implements Tomato's depth sensing capabilities using the power of Machine Learning&trade!
 *
 * Note: make sure to use the updated model [here](https://github.com/CDDelta/DBot/blob/master/models/pydnet.tflite).
 *
 * @author Matteo Poggi and Filippo Aleotti and Fabio Tosi and Stefano Mattoccia,
 * _Towards real-time unsupervised monocular depth estimation on CPU_,
 * IEEE/JRS Conference on Intelligent Robots and Systems (IROS), 2018.
 */
class Pydnet(val telemetry: PersistantTelemetry) : IVisionPipelineComponent {
    @Suppress("DEPRECATION")
    private val model = PydnetPP(File(Environment.getExternalStorageDirectory(), "tomato/pydnet.tflite"))

    init {
        model.prepare(Utils.Resolution.RES5)
    }

    override fun apply(mat: Mat, pipeline: VisionPipeline): Mat {
        // nb: mat is of type `CV_8UC4`
        val resizedMat = Mat(Utils.Resolution.RES5.height, Utils.Resolution.RES5.width, mat.type())
        resize(mat, resizedMat, Size(resizedMat.width().toDouble(), resizedMat.height().toDouble()), 0.0, 0.0, Imgproc.INTER_CUBIC)

        val inferredInverseDepth = model.doInference(resizedMat, Utils.Scale.FULL)
        for (y in 0..Utils.Resolution.RES5.height) {
            for (x in 0..Utils.Resolution.RES5.width) {
                val invDepth = (inferredInverseDepth[x + y * Utils.Resolution.RES5.height] * 255).toInt().toByte()
                resizedMat.put(y, x, byteArrayOf(invDepth, invDepth, invDepth, 255.toByte()))
            }
        }

        resize(resizedMat, mat, Size(mat.width().toDouble(), mat.height().toDouble()), 0.0, 0.0, Imgproc.INTER_NEAREST)
        return mat
    }
}
