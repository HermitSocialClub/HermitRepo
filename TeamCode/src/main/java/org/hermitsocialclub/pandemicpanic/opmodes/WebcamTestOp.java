package org.hermitsocialclub.pandemicpanic.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.hermitsocialclub.hydra.vision.DistanceToObjectDetector;
import org.hermitsocialclub.hydra.vision.VisionPipeline;
import org.hermitsocialclub.telecat.PersistantTelemetry;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@TeleOp(name = "WebCam Test")
public class WebcamTestOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webCam"), cameraMonitorViewId);
        webCam.openCameraDevice();

        webCam.setPipeline(new VisionPipeline(hardwareMap, new PersistantTelemetry(telemetry), new DistanceToObjectDetector()));
        webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Frame Count", webCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webCam.getFps()));
            telemetry.addData("Total frame time ms", webCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webCam.getCurrentPipelineMaxFps());
            telemetry.update();

            if (gamepad1.a) {
                webCam.stopStreaming();
            } else if (gamepad1.x) {
                webCam.pauseViewport();
            } else if (gamepad1.y) {
                webCam.resumeViewport();
            }

            sleep(100);
        }
    }

}