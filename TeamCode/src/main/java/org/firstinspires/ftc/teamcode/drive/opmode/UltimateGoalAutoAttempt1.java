package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.SkystoneVuforiaEngine;
import org.hermitsocialclub.telecat.PersistantTelemetry;

@Autonomous(name="Ultimate Goal Zone B Attempt 1")
public class UltimateGoalAutoAttempt1 extends LinearOpMode {
    Trajectory entirePath;
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();
        PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);
        SkystoneVuforiaEngine vuforiaEngine = SkystoneVuforiaEngine.get(telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap,vuforiaEngine);
        drive.setPoseEstimate(new Pose2d(-9,63,Math.toRadians(-90)));
        try {
            entirePath = drive.trajectoryBuilder(drive.getPoseEstimate(),0)
                    .splineTo(new Vector2d(-15.00,48.00),0)
                    .splineToLinearHeading(new Pose2d(0,0,0),-90)
                    .splineToLinearHeading(new Pose2d(48,36,0.00),90)
                    .splineTo(new Vector2d(-28,36),90)
                    .splineToConstantHeading(new Vector2d(-20,36),0)
                    .splineToConstantHeading(new Vector2d(-40,26),180)
                    .splineToSplineHeading(new Pose2d(20,36,180),0)
                    .splineToSplineHeading(new Pose2d(10,36,180),0)
                    .build();
        } catch (Exception e) {
            e.printStackTrace();
        }
        telemetry.setData("Acceleration", entirePath.acceleration(entirePath.duration()).toString());
        telemetry.setData("Duration", entirePath.duration());
        telemetry.setData("End", entirePath.end().toString());
        telemetry.setData("Start", entirePath.start().toString());
        telemetry.setData("Path", entirePath.getPath().toString());
        telemetry.setData("Velocity", entirePath.velocity(entirePath.duration()).toString());
        waitForStart();
        if (isStopRequested()) return;
        runtime.reset();
        drive.followTrajectory(entirePath);
    }


}
