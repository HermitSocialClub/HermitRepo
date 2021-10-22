package org.hermitsocialclub.tomato

import android.os.Environment
import org.hermitsocialclub.hydra.vision.IVisionPipelineComponent
import org.hermitsocialclub.hydra.vision.VisionPipeline
import org.hermitsocialclub.hydra.vision.util.asByteBuffer
import org.opencv.core.CvType
import org.opencv.core.Mat
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
import kotlin.math.min

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
        pipeline.telemetry.setData("mat.type()", CvType.typeToString(mat.type()))
        val inputType = when (mat.type()) {
            CvType.CV_8U -> DataType.UINT8
            CvType.CV_32F -> DataType.FLOAT32
            else -> throw IllegalArgumentException("Mat should have CV_8U or CV_32F type")
        }
        val inputBuffer = TensorBuffer.createFixedSize(intArrayOf(mat.height(), mat.width()), inputType)
        inputBuffer.loadBuffer(mat.asByteBuffer())
        inputImage.load(inputBuffer)

        // Process image
        // NormalizeOp arguments are from https://git.io/JKikt
        val cropSize = min(mat.width(), mat.height())
        ImageProcessor.Builder()
            .add(ResizeWithCropOrPadOp(cropSize, cropSize))
            .add(ResizeOp(imageSizeX, imageSizeY, ResizeMethod.NEAREST_NEIGHBOR))
            .add(NormalizeOp(115.0f, 58.0f))
            .build()
            .process(inputImage)

        // Run model!
        interpreter.run(inputImage.buffer, outputProbability.buffer.rewind())

        val outputType = when (outputProbability.dataType) {
            DataType.UINT8 -> CvType.CV_8U
            DataType.FLOAT32 -> CvType.CV_32F
            else -> throw IllegalArgumentException("Output tensor should have INT8 or FLOAT32 type")
        }
        return Mat(imageSizeY, imageSizeX, outputType, outputProbability.buffer)
    }

    // hopefully someday KT-83 will be implemented
    override fun close() {
        interpreter.close()
    }
}
