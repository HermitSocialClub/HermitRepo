package org.hermitsocialclub.opmodes

import android.os.Environment
import com.qualcomm.robotcore.util.ElapsedTime
import org.hermitsocialclub.hydra.vision.StaccDetecc
import org.hermitsocialclub.hydra.vision.SubmatRenderer
import org.hermitsocialclub.hydra.vision.VisionPipeline
import org.hermitsocialclub.hydra.vision.util.VisionUtils
import org.hermitsocialclub.telecat.PersistantTelemetry
import org.openftc.easyopencv.OpenCvCamera
import java.io.File
import kotlin.math.abs
import kotlin.math.floor

class SubmatSetupOp : AbstractVisionTestOp() {

    companion object {
        @JvmField
        val SUBMAT_CONFIG = File(
            Environment.getExternalStorageDirectory().path + File.separator + "stacc-detecc-submat.config"
        )
    }

    var saveTimer = ElapsedTime()
    var submat = VisionUtils.loadRectFromFile(SUBMAT_CONFIG)
    val staccDetecc = StaccDetecc()
    val submatRenderer = SubmatRenderer(submat)

    override fun buildPipeline(telemetry: PersistantTelemetry): VisionPipeline {
        return VisionPipeline(hardwareMap, telemetry, staccDetecc, submatRenderer)
    }

    override fun runLoop(telemetry: PersistantTelemetry, camera: OpenCvCamera, pipeline: VisionPipeline) {
        telemetry.setData("FPS", "%.2f", camera.fps)
        telemetry.setData("", "Use left stick to configure position and right stick to configure size. Press A to save.")
        telemetry.setData("Position", "%d %d", submat.x, submat.y)
        telemetry.setData("Size", "%d %d", submat.width, submat.height)

        if(abs(gamepad1.left_stick_x) >= 0.1) {
            submat.x += floor(gamepad1.left_stick_x * 10.0).toInt()
        }
        if(abs(gamepad1.left_stick_y) >= 0.1) {
            submat.y += floor(gamepad1.left_stick_y * 10.0).toInt()
        }
        if(abs(gamepad1.right_stick_x) >= 0.1) {
            submat.width += floor(gamepad1.right_stick_x * 10.0).toInt()
        }
        if(abs(gamepad1.right_stick_y) >= 0.1) {
            submat.height += floor(gamepad1.right_stick_y * 10.0).toInt()
        }
        staccDetecc.config.submat = submat
        submatRenderer.submat = submat

        if(gamepad1.a) {
            VisionUtils.saveRectToFile(submat, SUBMAT_CONFIG)
            telemetry.setData("Status", "Saved!")
            saveTimer.reset()
        }
        if(saveTimer.seconds() > 3) {
            telemetry.removeData("Status")
        }
    }
}