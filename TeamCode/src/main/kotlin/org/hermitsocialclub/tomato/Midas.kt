package org.hermitsocialclub.tomato

import android.os.Environment
import org.hermitsocialclub.hydra.vision.IVisionPipelineComponent
import org.hermitsocialclub.hydra.vision.VisionPipeline
import org.hermitsocialclub.telecat.PersistantTelemetry
import org.hermitsocialclub.util.Profiler
import org.opencv.core.Core.normalize
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Rect
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc
import org.opencv.imgproc.Imgproc.*
import org.tensorflow.lite.DataType
import org.tensorflow.lite.Interpreter
import org.tensorflow.lite.support.common.ops.NormalizeOp
import org.tensorflow.lite.support.image.ImageProcessor
import org.tensorflow.lite.support.image.TensorImage
import org.tensorflow.lite.support.image.ops.ResizeOp
import org.tensorflow.lite.support.image.ops.ResizeOp.ResizeMethod
import org.tensorflow.lite.support.image.ops.ResizeWithCropOrPadOp
import org.tensorflow.lite.support.tensorbuffer.TensorBuffer
import java.io.File
import java.nio.ByteBuffer
import kotlin.math.min

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
                setNumThreads(4)
                setUseXNNPACK(true)
                telemetry.setData("Tflite Backend", "CPU")
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
        val inputSize = mat.size()
        val matWidth = mat.width()
        val matHeight = mat.height()
        val cropSize = min(matWidth, matHeight)

        val croppedMat = mat.submat(Rect(
            (matWidth - cropSize) / 2,
            (matHeight - cropSize) / 2,
            cropSize,
            cropSize
        )).clone()
        cvtColor(croppedMat, croppedMat, COLOR_RGBA2BGR)

        val resizedMat = Mat(imageSizeY, imageSizeX, croppedMat.type())
        resize(croppedMat, resizedMat, Size(imageSizeY.toDouble(), imageSizeX.toDouble()), 0.0, 0.0, INTER_NEAREST)

        val inputBuffer = TensorBuffer.createFixedSize(intArrayOf(imageSizeY, imageSizeX, 3), DataType.UINT8)
        val inputSizeInBytes = (resizedMat.total() * resizedMat.channels()).toInt()
        val badHackBuf = if (this.badHack?.size == inputSizeInBytes) {
            this.badHack!!
        } else {
            val newBuf = ByteArray(inputSizeInBytes)
            this.badHack = newBuf
            newBuf
        }
        resizedMat.get(0, 0, badHackBuf)
        inputBuffer.loadBuffer(ByteBuffer.wrap(badHackBuf))

        // Run model!
        profiler.begin("interpreter.run")
        interpreter.run(inputBuffer, outputProbability.buffer.rewind())
        profiler.end()

        outputProbability.buffer.rewind()
        val modelOutMat = Mat(imageSizeY, imageSizeX, CvType.CV_8UC(3), outputProbability.buffer)
        val outBgrMat = Mat()
        resize(modelOutMat, outBgrMat, inputSize)
        cvtColor(outBgrMat, mat, COLOR_BGR2RGBA)
        return mat
    }

    // hopefully someday KT-83 will be implemented
    override fun close() {
        interpreter.close()
    }
}
