package org.hermitsocialclub.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.hermitsocialclub.hydra.vision.DistanceToObjectDetector
import org.hermitsocialclub.hydra.vision.VisionPipeline
import org.hermitsocialclub.telecat.PersistantTelemetry
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

@TeleOp(name = "Contour Test")
class ContourTestOp : LinearOpMode() {

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
        val pipeline = VisionPipeline(telemetry, DistanceToObjectDetector())
        phoneCam.setPipeline(pipeline)
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)

        waitForStart()

        while (opModeIsActive()) {
            telemetry.setData("Frame Count", phoneCam.frameCount)
            telemetry.setData("FPS", "%.2f", phoneCam.fps)
            telemetry.setData("Canny Lower", pipeline.cannyLowerThreshold)
            telemetry.setData("Canny Upper", pipeline.cannyUpperThreshold)
            // telemetry.setData("Total frame time ms", phoneCam.totalFrameTimeMs)
            // telemetry.setData("Pipeline time ms", phoneCam.pipelineTimeMs)
            // telemetry.setData("Overhead time ms", phoneCam.overheadTimeMs)
            // telemetry.setData("Theoretical max FPS", phoneCam.currentPipelineMaxFps)

            if (gamepad1.dpad_up) {
                pipeline.cannyUpperThreshold = pipeline.cannyUpperThreshold + 5
            } else if (gamepad1.dpad_down) {
                pipeline.cannyUpperThreshold = pipeline.cannyUpperThreshold - 5
            }
            if (gamepad1.dpad_left) {
                pipeline.cannyLowerThreshold = pipeline.cannyLowerThreshold + 5
            } else if (gamepad1.dpad_right) {
                pipeline.cannyLowerThreshold = pipeline.cannyLowerThreshold - 5
            }

            sleep(10)
        }
    }
}