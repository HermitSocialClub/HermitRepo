package org.hermitsocialclub.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.hermitsocialclub.hydra.vision.VisionPipeline
import org.hermitsocialclub.telecat.PersistantTelemetry
import org.hermitsocialclub.util.Jukebox
import org.openftc.easyopencv.OpenCvCamera
import java.io.PrintWriter
import java.io.StringWriter

abstract class AbstractVisionTestOp : LinearOpMode() {

    override fun runOpMode() {
        val telemetry = PersistantTelemetry(telemetry)

        try {
            val pipeline = buildPipeline(telemetry)

            while(!isStarted){
                runLoop(telemetry,pipeline.camera,pipeline)
            }

            waitForStart()

            while (opModeIsActive()) {
                runLoop(telemetry, pipeline.camera, pipeline)
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