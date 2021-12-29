package org.hermitsocialclub.tomato

import android.os.Environment
import org.hermitsocialclub.hydra.vision.IVisionPipelineComponent
import org.hermitsocialclub.hydra.vision.VisionPipeline
import org.hermitsocialclub.telecat.PersistantTelemetry
import org.hermitsocialclub.util.Profiler
import org.opencv.core.Core.NORM_MINMAX
import org.opencv.core.Core.normalize
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc.INTER_CUBIC
import org.opencv.imgproc.Imgproc.INTER_NEAREST
import org.opencv.imgproc.Imgproc.resize
import org.tensorflow.lite.Interpreter
import unibo.cvlab.pydnet.TensorflowLiteModel
import unibo.cvlab.pydnet.Utils.Resolution
import unibo.cvlab.pydnet.Utils.Scale
import java.io.File

/**
 * ## Mixing Datasets for Zero-shot Cross-Dataset Transfer
 *
 * A wrapper around the [MiDaS TFLite model](https://github.com/isl-org/MiDaS)
 * which implements Tomato's depth sensing capabilities using the power of
 * Machine Learning&trade;!
 *
 * @author Ranftl and Katrin Lasinger and David Hafner and Konrad Schindler and Vladlen Koltun,
 * _Towards Robust Monocular Depth Estimation: Mixing Datasets for Zero-shot Cross-dataset Transfer_,
 * IEEE Transactions on Pattern Analysis and Machine Intelligence (TPAMI), 2020.
 */
class Midas(telemetry: PersistantTelemetry) : IVisionPipelineComponent {
    @Suppress("DEPRECATION")
    private val model = MidasModel(File(Environment.getExternalStorageDirectory(), "tomato/model_opt.tflite"))
    private val profiler = Profiler("MIDAS", telemetry)

    init {
        model.prepare(Resolution.SQUARE_256)
    }

    override fun apply(mat: Mat, pipeline: VisionPipeline): Mat {
        profiler.begin("setup")
        val resizedMat = Mat(Resolution.SQUARE_256.height, Resolution.SQUARE_256.width, mat.type())
        resize(mat, resizedMat, Size(resizedMat.width().toDouble(), resizedMat.height().toDouble()), 0.0, 0.0, INTER_CUBIC)

        profiler.swap("inference")
        val inferredRelativeDepth = model.doInference(resizedMat, Scale.FULL)
        val inferredRelativeDepthMat = Mat(Resolution.SQUARE_256.height, Resolution.SQUARE_256.width, CvType.CV_32FC(4))

        profiler.swap("post-process")
        for (y in 0 until Resolution.SQUARE_256.height) {
            for (x in 0 until Resolution.SQUARE_256.width) {
                val depth = inferredRelativeDepth[x + y * Resolution.SQUARE_256.width]
                inferredRelativeDepthMat.put(y, x, floatArrayOf(depth, depth, depth, 1.0F))
            }
        }
        normalize(inferredRelativeDepthMat, resizedMat, 0.0, 255.0, NORM_MINMAX, resizedMat.type())
        resize(resizedMat, mat, Size(mat.width().toDouble(), mat.height().toDouble()), 0.0, 0.0, INTER_NEAREST)
        profiler.end()

        return mat
    }
}

class MidasModel(modelFile: File) : TensorflowLiteModel("MiDaS", modelFile, false) {
    init {
        addInputNode("image", "Const")
        addOutputNodes(Scale.FULL, "midas_net_custom/sequential/re_lu_9/Relu")
        addValidResolution(Resolution.SQUARE_256)
    }

    override fun configureInterpreter(options: Interpreter.Options) {
        options.setUseXNNPACK(true)
        options.setNumThreads(4)
    }
}
