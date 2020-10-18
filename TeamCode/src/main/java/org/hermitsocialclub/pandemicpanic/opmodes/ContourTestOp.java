package org.hermitsocialclub.pandemicpanic.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.hermitsocialclub.hydra.vision.DistanceToObjectDetector;
import org.hermitsocialclub.hydra.vision.VisionPipeline;
import org.hermitsocialclub.telecat.PersistantTelemetry;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp(name = "Contour Test")
public class ContourTestOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();

        VisionPipeline pipeline = new VisionPipeline(new PersistantTelemetry(telemetry), new DistanceToObjectDetector());
        phoneCam.setPipeline(pipeline);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();

            if (gamepad1.dpad_up) {
                pipeline.setCannyUpperThreshold(pipeline.getCannyUpperThreshold() + 5);
            } else if (gamepad1.dpad_down) {
                pipeline.setCannyUpperThreshold(pipeline.getCannyUpperThreshold() - 5);
            }
            if (gamepad1.dpad_left) {
                pipeline.setCannyLowerThreshold(pipeline.getCannyLowerThreshold() + 5);
            } else if (gamepad1.dpad_right) {
                pipeline.setCannyLowerThreshold(pipeline.getCannyLowerThreshold() - 5);
            }

            sleep(10);
        }
    }

}