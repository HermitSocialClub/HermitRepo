package org.hermitsocialclub.drive.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.telecat.PersistantTelemetry;
import org.hermitsocialclub.legacy.SkystoneVuforiaEngine;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        PersistantTelemetry pt = new PersistantTelemetry(telemetry);
        SkystoneVuforiaEngine vuforiaEngine = SkystoneVuforiaEngine.get(pt);
        BaselineMecanumDrive drive = new BaselineMecanumDrive(hardwareMap, pt);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        drive.turn(Math.toRadians(ANGLE));
    }
}
