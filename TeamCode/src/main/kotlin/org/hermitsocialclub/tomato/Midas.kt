package org.hermitsocialclub.tomato

import android.os.Environment
import com.qualcomm.robotcore.util.RobotLog
import org.hermitsocialclub.hydra.vision.IVisionPipelineComponent
import org.hermitsocialclub.hydra.vision.VisionPipeline
import org.hermitsocialclub.hydra.vision.util.asByteBuffer
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc
import org.opencv.imgproc.Imgproc.cvtColor
import org.tensorflow.lite.DataType
import org.tensorflow.lite.Interpreter
import org.tensorflow.lite.gpu.CompatibilityList
import org.tensorflow.lite.gpu.GpuDelegate
import org.tensorflow.lite.support.common.ops.NormalizeOp
import org.tensorflow.lite.support.image.ImageProcessor
import org.tensorflow.lite.support.image.TensorImage
import org.tensorflow.lite.support.image.ops.ResizeOp
import org.tensorflow.lite.support.image.ops.ResizeOp.ResizeMethod
import org.tensorflow.lite.support.image.ops.ResizeWithCropOrPadOp
import org.tensorflow.lite.support.tensorbuffer.TensorBuffer
import java.io.File
import java.lang.IllegalArgumentException
import java.nio.ByteBuffer
import kotlin.math.min

private const val TAG: String = "MIDAS DEBUGGING"

/**
 * ## Mixing Datasets for Zero-shot Cross-Dataset Transfer
 *
 * A wrapper around the [MiDaS TFLite model](https://github.com/isl-org/MiDaS)
 * which implements Tomato's depth sensing capabilities using the power of
 * Machine Learning&trade!
 *
 * @author Ranftl and Katrin Lasinger and David Hafner and Konrad Schindler and Vladlen Koltun,
 * _Towards Robust Monocular Depth Estimation: Mixing Datasets for Zero-shot Cross-dataset Transfer_,
 * IEEE Transactions on Pattern Analysis and Machine Intelligence (TPAMI), 2020.
 */
class Midas : AutoCloseable, IVisionPipelineComponent {

    private val interpreter: Interpreter
    private val imageSizeX: Int
    private val imageSizeY: Int
    private val inputImage: TensorImage
    private val outputProbability: TensorBuffer

    init {
        @Suppress("DEPRECATION")
        interpreter = Interpreter(
            File(Environment.getExternalStorageDirectory(), "tomato/model_opt.tflite"),
            Interpreter.Options().apply {
                val compatList = CompatibilityList()
                if (compatList.isDelegateSupportedOnThisDevice) {
                    val delegateOptions = compatList.bestOptionsForThisDevice
                    this.addDelegate(GpuDelegate(delegateOptions))
                } else {
                    this.setNumThreads(1)
                }
            }
        )

        val inputTensor = interpreter.getInputTensor(0)
        val outputTensor = interpreter.getOutputTensor(0)

        val imageShape = inputTensor.shape()
        if (imageShape[1] != imageShape[2]) {
            imageSizeX = imageShape[3]
            imageSizeY = imageShape[2]
        } else {
            imageSizeX = imageShape[2]
            imageSizeY = imageShape[1]
        }

        inputImage = TensorImage(inputTensor.dataType())
        outputProbability = TensorBuffer.createFixedSize(outputTensor.shape(), outputTensor.dataType())
    }

    override fun apply(mat: Mat, pipeline: VisionPipeline): Mat {
        // Load image
        val bgrMat = Mat()
        pipeline.telemetry.setData(TAG, "cvtColor")
        cvtColor(mat, bgrMat, Imgproc.COLOR_RGBA2BGR)
        pipeline.telemetry.setData(TAG, "TensorBuffer.createFixedSize")
        val inputBuffer = TensorBuffer.createFixedSize(intArrayOf(mat.height(), mat.width(), 3), DataType.UINT8)
        pipeline.telemetry.setData(TAG, "badHackPlsRemove")
        val badHackPlsRemoveFloatBuffer = ByteArray((bgrMat.total() * bgrMat.channels()).toInt())
        bgrMat.get(0, 0, badHackPlsRemoveFloatBuffer)
        pipeline.telemetry.setData(TAG, "loadBuffer")
        inputBuffer.loadBuffer(ByteBuffer.wrap(badHackPlsRemoveFloatBuffer))
        pipeline.telemetry.setData(TAG, "load")
        inputImage.load(inputBuffer)

        // Process image
        // NormalizeOp arguments are from https://git.io/JKikt
        val cropSize = min(mat.width(), mat.height())
        pipeline.telemetry.setData(TAG, "resize and normalize")
        ImageProcessor.Builder()
            .add(ResizeWithCropOrPadOp(cropSize, cropSize))
            .add(ResizeOp(imageSizeX, imageSizeY, ResizeMethod.NEAREST_NEIGHBOR))
            .add(NormalizeOp(115.0f, 58.0f))
            .build()
            .process(inputImage)

        // Run model!
        pipeline.telemetry.setData(TAG, "interpreter.run")
        interpreter.run(inputImage.buffer, outputProbability.buffer.rewind())

        val outBgrMat = Mat(imageSizeY, imageSizeX, CvType.CV_8UC(3), outputProbability.buffer)
        pipeline.telemetry.setData(TAG, "cvtColor 2")
        cvtColor(outBgrMat, mat, Imgproc.COLOR_BGR2RGBA)
        return mat
    }

    // hopefully someday KT-83 will be implemented
    override fun close() {
        interpreter.close()
    }
}
