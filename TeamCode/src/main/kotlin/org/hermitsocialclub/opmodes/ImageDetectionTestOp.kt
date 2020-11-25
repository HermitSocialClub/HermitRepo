package org.hermitsocialclub.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.hermitsocialclub.hydra.vision.BetterVuforia
import org.hermitsocialclub.hydra.vision.VisionPipeline
import org.hermitsocialclub.telecat.PersistantTelemetry
import org.openftc.easyopencv.OpenCvCamera

@TeleOp(name = "ImageDetectionTestOp")
class ImageDetectionTestOp : AbstractVisionTestOp() {

    override fun buildPipeline(telemetry: PersistantTelemetry): VisionPipeline {
        return VisionPipeline(hardwareMap, telemetry, BetterVuforia())
    }

    override fun runLoop(telemetry: PersistantTelemetry, camera: OpenCvCamera, pipeline: VisionPipeline) {
        telemetry.setData("Frame Count", camera.frameCount)
        telemetry.setData("FPS", "%.2f", camera.fps)

        sleep(10)
    }

}