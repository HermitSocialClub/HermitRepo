package org.hermitsocialclub.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.hermitsocialclub.hydra.vision.VisionPipeline
import org.hermitsocialclub.telecat.PersistantTelemetry
import org.hermitsocialclub.util.Jukebox
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import java.io.PrintWriter
import java.io.StringWriter

abstract class AbstractVisionTestOp : LinearOpMode() {

    override fun runOpMode() {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        val phoneCam: OpenCvCamera = OpenCvCameraFactory.getInstance()
            .createWebcam(hardwareMap.get(WebcamName::class.java, "webCam"), cameraMonitorViewId)
        phoneCam.openCameraDevice()

        val telemetry = PersistantTelemetry(telemetry)

        try {
            val pipeline = buildPipeline(telemetry)
            phoneCam.setPipeline(pipeline)
            phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)

            waitForStart()

            while (opModeIsActive()) {
                runLoop(telemetry, phoneCam, pipeline)
            }
        } catch (t: Throwable) {
            val writer = StringWriter()
            t.printStackTrace(PrintWriter(writer, true))
            Jukebox.setTelemetryWarning(writer.toString())
        }
    }

    protected abstract fun buildPipeline(telemetry: PersistantTelemetry): VisionPipeline

    protected abstract fun runLoop(telemetry: PersistantTelemetry, camera: OpenCvCamera, pipeline: VisionPipeline)

}