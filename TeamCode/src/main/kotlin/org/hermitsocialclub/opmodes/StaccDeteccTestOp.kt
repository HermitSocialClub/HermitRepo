package org.hermitsocialclub.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import android.os.Environment
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.hermitsocialclub.hydra.vision.StaccDetecc
import org.hermitsocialclub.hydra.vision.VisionPipeline
import org.hermitsocialclub.hydra.vision.util.CameraConfig
import org.hermitsocialclub.telecat.PersistantTelemetry
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfDouble
import org.openftc.easyopencv.OpenCvCamera
import java.io.File
import kotlin.math.max
import kotlin.math.min

@TeleOp(name = "StaccDeteccTestOp")
class StaccDeteccTestOp : AbstractVisionTestOp() {

    companion object {
        @JvmField
        val CAMERA_CONFIG_FILE = File(
            Environment.getExternalStorageDirectory().path + File.separator + "camera_info.xml"
        )

        @JvmField
        val CAMERA_CONFIG: CameraConfig

        init {
            val cameraMatrix = Mat(3, 3, CvType.CV_64F)
            cameraMatrix.put(0, 0,
                1.3606600164833324e+03, 0.0, 9.5950000000000000e+02,
                0.0, 1.3606600164833324e+03, 5.3950000000000000e+02,
                0.0, 0.0, 1.0
            )
            val distortionMatrix = MatOfDouble(
                -1.5325995853909627e-02, -7.6109310711621053e-02, 0.0, 0.0,
                5.4888141489714659e-02
            )
            CAMERA_CONFIG = CameraConfig(cameraMatrix, distortionMatrix)
        }
    }

    val staccDetecc = StaccDetecc(cameraConfig = CAMERA_CONFIG)

    override fun buildPipeline(telemetry: PersistantTelemetry): VisionPipeline {
        return VisionPipeline(hardwareMap, telemetry, staccDetecc)
    }

    override fun runLoop(telemetry: PersistantTelemetry, camera: OpenCvCamera, pipeline: VisionPipeline) {
        telemetry.setData("Frame Count", camera.frameCount)
        telemetry.setData("FPS", "%.2f", camera.fps)
        telemetry.setData("topThreshold", staccDetecc.config.topThreshold)

        if (gamepad1.dpad_down) {
            staccDetecc.config.topThreshold -= 1.0
        } else if (gamepad1.dpad_up) {
            staccDetecc.config.topThreshold += 1.0
        }
        staccDetecc.config.topThreshold = min(max(0.0, staccDetecc.config.topThreshold), 254.0)
    }
}