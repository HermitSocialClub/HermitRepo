package org.hermitsocialclub.hydra.vision

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.hermitsocialclub.telecat.PersistantTelemetry
import org.opencv.core.Mat
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline

class VisionPipeline(
    val hardwareMap: HardwareMap,
    val telemetry: PersistantTelemetry,
    vararg components: IVisionPipelineComponent
) : OpenCvPipeline() {

    val camera: OpenCvCamera
    val pipeline: List<IVisionPipelineComponent> = listOf(*components)
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

        pipeline.forEach {
            it.init(this)
        }

        camera.setPipeline(this)
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
    }

    override fun processFrame(input: Mat): Mat {
        var mat = input
        pipeline.forEach { mat = it.apply(mat, this) }
        return mat
    }

}
