package org.hermitsocialclub.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.hermitsocialclub.hydra.vision.VisionPipeline
import org.hermitsocialclub.telecat.PersistantTelemetry
import org.hermitsocialclub.tomato.BarcodeDetect
import org.openftc.easyopencv.OpenCvCamera

@TeleOp(name = "BarcodeDetectOp")
class BarcodeDetectOp : AbstractVisionTestOp() {
	val detector = BarcodeDetect(true)
    override fun buildPipeline(telemetry: PersistantTelemetry): VisionPipeline {
        return VisionPipeline(hardwareMap, telemetry, detector)
    }

    override fun runLoop(telemetry: PersistantTelemetry, camera: OpenCvCamera, pipeline: VisionPipeline) {
        telemetry.setData("Frame Count", camera.frameCount)
        telemetry.setData("FPS", "%.2f", camera.fps)
    }
}
