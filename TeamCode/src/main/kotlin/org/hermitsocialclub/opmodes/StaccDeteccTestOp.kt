package org.hermitsocialclub.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import android.os.Environment
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.hermitsocialclub.hydra.vision.StaccDetecc
import org.hermitsocialclub.hydra.vision.VisionPipeline
import org.hermitsocialclub.hydra.vision.util.CameraConfig
import org.hermitsocialclub.telecat.PersistantTelemetry
import org.openftc.easyopencv.OpenCvCamera
import java.io.File

@TeleOp(name = "StaccDeteccTestOp")
class StaccDeteccTestOp : AbstractVisionTestOp() {

    companion object {
        @JvmField
        val CAMERA_CONFIG = File(
            Environment.getExternalStorageDirectory().path + File.separator + "camera_info.xml"
        )
    }

    override fun buildPipeline(telemetry: PersistantTelemetry): VisionPipeline {
        val cameraConfig = CameraConfig.loadFromFile(CAMERA_CONFIG)
        return VisionPipeline(hardwareMap, telemetry, StaccDetecc(cameraConfig = cameraConfig))
    }

    override fun runLoop(telemetry: PersistantTelemetry, camera: OpenCvCamera, pipeline: VisionPipeline) {
        telemetry.setData("Frame Count", camera.frameCount)
        telemetry.setData("FPS", "%.2f", camera.fps)
    }
}