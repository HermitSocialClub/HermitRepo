package org.hermitsocialclub.tomato

import android.os.Environment
import org.hermitsocialclub.hydra.vision.IVisionPipelineComponent
import org.hermitsocialclub.hydra.vision.VisionPipeline
import org.hermitsocialclub.telecat.PersistantTelemetry
import org.hermitsocialclub.util.Profiler
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc
import org.opencv.imgproc.Imgproc.cvtColor
import org.opencv.imgproc.Imgproc.resize
import org.tensorflow.lite.DataType
import org.tensorflow.lite.Interpreter
import org.tensorflow.lite.gpu.CompatibilityList
import org.tensorflow.lite.gpu.GpuDelegate
import org.tensorflow.lite.support.image.ImageProcessor
import org.tensorflow.lite.support.image.TensorImage
import org.tensorflow.lite.support.image.ops.ResizeOp
import org.tensorflow.lite.support.image.ops.ResizeOp.ResizeMethod
import org.tensorflow.lite.support.image.ops.ResizeWithCropOrPadOp
import org.tensorflow.lite.support.tensorbuffer.TensorBuffer
import java.io.File
import java.nio.ByteBuffer
import kotlin.math.min
import kotlin.system.measureTimeMillis

private const val TAG: String = "MIDAS"

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
class Midas(telemetry: PersistantTelemetry) : AutoCloseable, IVisionPipelineComponent {

    private val profiler = Profiler(TAG, telemetry)
    private val interpreter: Interpreter
    private val imageSizeX: Int
    private val imageSizeY: Int
    private val inputImage: TensorImage
    private val outputProbability: TensorBuffer
    private var badHack: ByteArray? = null

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
        profiler.begin("input setup")
        // Load image
        val inputSize = mat.size()
        val bgrMat = Mat()
        cvtColor(mat, bgrMat, Imgproc.COLOR_RGBA2BGR)
        profiler.swap("badHackBuf")
        val inputBuffer = TensorBuffer.createFixedSize(intArrayOf(mat.height(), mat.width(), 3), DataType.UINT8)
        val inputSizeInBytes = (bgrMat.total() * bgrMat.channels()).toInt()
        val badHackBuf = if(this.badHack?.size == inputSizeInBytes) {
            this.badHack!!
        } else {
            val newBuf = ByteArray(inputSizeInBytes)
            this.badHack = newBuf
            newBuf
        }
        profiler.swap("bgrMat.get")
        bgrMat.get(0, 0, badHackBuf)
        profiler.swap("load")
        inputBuffer.loadBuffer(ByteBuffer.wrap(badHackBuf))
        inputImage.load(inputBuffer)

        // Process image
        // NormalizeOp arguments are from https://git.io/JKikt
        profiler.swap("processor init")
        val cropSize = min(mat.width(), mat.height())
        ImageProcessor.Builder()
            .add(ResizeWithCropOrPadOp(cropSize, cropSize))
            .add(ResizeOp(imageSizeX, imageSizeY, ResizeMethod.NEAREST_NEIGHBOR))
        //  .add(NormalizeOp(115.0f, 58.0f))
            .build()
            .process(inputImage)

        // Run model!
        profiler.swap("interpreter.run")
        interpreter.run(inputImage.buffer, outputProbability.buffer.rewind())

        profiler.swap("modelOutMat")
        val modelOutMat = Mat(imageSizeY, imageSizeX, CvType.CV_8UC(3), outputProbability.buffer)
        val outBgrMat = Mat()
        resize(modelOutMat, outBgrMat, inputSize)
        cvtColor(outBgrMat, mat, Imgproc.COLOR_BGR2RGBA)
        profiler.end()
        return mat
    }

    // hopefully someday KT-83 will be implemented
    override fun close() {
        interpreter.close()
    }
}
