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
    private val pipelineMutex = Object()
    var pipeline: List<IVisionPipelineComponent> = listOf(*components)
        set(value) {
            synchronized(pipelineMutex) {
                field = value
            }
        }

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
        camera.openCameraDevice()

        synchronized(pipelineMutex) {
            pipeline.forEach {
                it.init(this)
            }
        }

        camera.setPipeline(this)
        start()
    }
    override fun processFrame(input: Mat): Mat {
        var mat = input
        pipeline.forEach { mat = it.apply(mat, this) }
        return mat
    }

    fun start() {
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
    }

    fun pause() {
        camera.stopStreaming()
    }

    override fun close() {
        camera.closeCameraDevice()
    }

}
