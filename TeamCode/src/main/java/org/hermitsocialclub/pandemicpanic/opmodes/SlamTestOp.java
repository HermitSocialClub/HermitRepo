package org.hermitsocialclub.pandemicpanic.opmodes;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import org.hermitsocialclub.telecat.PersistantTelemetry;

@TeleOp (name = "SlamTestOp")
public class SlamTestOp extends LinearOpMode {

    Pose2d startingPose = new Pose2d(1, 1, new Rotation2d());

    Transform2d cameraToRobot = new Transform2d();

    double encoderMeasurementCovariance = 0.8;

    T265Camera slamra;
    T265Camera.CameraUpdate update;

    PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);

    private static T265Camera slamJam;

    @Override
    public void runOpMode() throws InterruptedException {
        slamra = new T265Camera(cameraToRobot, encoderMeasurementCovariance, hardwareMap.appContext);

        slamra.setPose(startingPose);

        slamra.start();
        telemetry.setDebug("started","slamra");

        waitForStart();

        while (opModeIsActive()) {
            update = slamra.getLastReceivedCameraUpdate();
            telemetry.setDebug("Pose",update.pose.toString());

        }
    }
}
