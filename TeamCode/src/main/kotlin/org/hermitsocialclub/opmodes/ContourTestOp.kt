package org.hermitsocialclub.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.hermitsocialclub.hydra.vision.DistanceToObjectDetector
import org.hermitsocialclub.hydra.vision.VisionPipeline
import org.hermitsocialclub.telecat.PersistantTelemetry
import org.openftc.easyopencv.OpenCvCamera
@Disabled
@TeleOp(name = "Contour Test")
class ContourTestOp : AbstractVisionTestOp() {

    override fun buildPipeline(telemetry: PersistantTelemetry): VisionPipeline {
        return VisionPipeline(hardwareMap, telemetry, DistanceToObjectDetector())
    }

    override fun runLoop(telemetry: PersistantTelemetry, camera: OpenCvCamera, pipeline: VisionPipeline) {
        telemetry.setData("Frame Count", camera.frameCount)
        telemetry.setData("FPS", "%.2f", camera.fps)
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