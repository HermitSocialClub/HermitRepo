package org.hermitsocialclub.hydra.vision

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.hermitsocialclub.telecat.PersistantTelemetry
import org.opencv.core.Mat
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline
import java.io.Closeable

class VisionPipeline(
    val hardwareMap: HardwareMap,
    val telemetry: PersistantTelemetry,
    vararg components: IVisionPipelineComponent
) : OpenCvPipeline(), Closeable {

    val camera: OpenCvCamera
    var pipeline: List<IVisionPipelineComponent> = listOf(*components)
        set(value) {
            synchronized(pipelineMutex) {
                field = value
            }
        }
    private val pipelineMutex = Object()
    private var initYet = false
    private var initError: Throwable? = null

    var cannyLowerThreshold: Double = 35.0
    var cannyUpperThreshold: Double = 125.0

    init {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        camera = OpenCvCameraFactory.getInstance()
            .createWebcam(hardwareMap.get(WebcamName::class.java, "webCam"), cameraMonitorViewId)

        camera.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                synchronized(pipelineMutex) {
                    pipeline.forEach {
                        it.init(this@VisionPipeline)
                    }
                    initYet = true
                }

                camera.setPipeline(this@VisionPipeline)
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {
                synchronized(pipelineMutex) {
                    initError = RuntimeException("Couldn't open camera device: $errorCode")
                }
            }
        })
    }

    override fun processFrame(input: Mat): Mat = synchronized(pipelineMutex) {
        return if (initYet) {
            // pipeline initialized, check if there was an
            // error before proceeding
            if (initError != null) {
                throw initError!!
            }

            // no error, process the frame using our pipeline
            var mat = input
            pipeline.forEach { mat = it.apply(mat, this) }
            mat
        } else {
            // pipeline not initialized yet
            input
        }
    }

    override fun close() = synchronized(pipelineMutex) {
        camera.closeCameraDevice()
    }
}
