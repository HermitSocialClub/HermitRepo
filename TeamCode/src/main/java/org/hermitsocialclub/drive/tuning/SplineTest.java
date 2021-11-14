package org.hermitsocialclub.drive.tuning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.telecat.PersistantTelemetry;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "SplineTest",group = "drive")
public class SplineTest extends LinearOpMode {

    PersistantTelemetry pt = new PersistantTelemetry(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        BaselineMecanumDrive drive = new BaselineMecanumDrive(hardwareMap,pt);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();

        drive.followTrajectory(traj);

        sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );
    }
}