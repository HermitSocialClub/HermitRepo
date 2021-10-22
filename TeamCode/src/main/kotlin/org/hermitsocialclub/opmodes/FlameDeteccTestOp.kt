package org.hermitsocialclub.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.hermitsocialclub.hydra.vision.StaccDetecc
import org.hermitsocialclub.hydra.vision.VisionPipeline
import org.hermitsocialclub.telecat.PersistantTelemetry
import org.opencv.core.Scalar
import org.openftc.easyopencv.OpenCvCamera

@TeleOp(name = "FlameDeteccTestOp")
class FlameDeteccTestOp : AbstractVisionTestOp() {
    val staccDetecc = StaccDetecc(
        StaccDetecc.StaccConfig().apply {
            lowerYellow = Scalar(160.0, 100.0, 100.0)
            upperYellow = Scalar(320.0, 255.0, 255.0)
        }
    )

    override fun buildPipeline(telemetry: PersistantTelemetry): VisionPipeline {
        return VisionPipeline(hardwareMap, telemetry, staccDetecc)
    }

    override fun runLoop(telemetry: PersistantTelemetry, camera: OpenCvCamera, pipeline: VisionPipeline) {
        telemetry.setData("Frame Count", camera.frameCount)
        telemetry.setData("FPS", "%.2f", camera.fps)
    }
}
