package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Meet3Bot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.SkystoneVuforiaEngine;
import org.hermitsocialclub.telecat.PersistantTelemetry;

@Disabled
@Autonomous(name = "RR Auto Skystone AC3")
public class SkystoneAutoAttemptAbsoluteCoordinates3 extends LinearOpMode {
    Trajectory t3;
    Trajectory t5;
    Trajectory t2;
    Trajectory t1;
    Trajectory t6;
    Trajectory t7;
    Trajectory t8;
    Trajectory t9;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();
        PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);
        SkystoneVuforiaEngine vuforiaEngine = SkystoneVuforiaEngine.get(telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap,vuforiaEngine);
        TrajectoryGroupConfig tgc = new TrajectoryGroupConfig(drive.constraints.maxVel,drive.constraints.maxAccel,drive.constraints.maxAngVel,drive.constraints.maxAngAccel,15,10.75, TrajectoryGroupConfig.DriveType.MECANUM, Meet3Bot.TRACK_WIDTH, Meet3Bot.TRACK_WIDTH,1.0);
        drive.setPoseEstimate(new Pose2d(-9,63,Math.toRadians(-90)));
        try {
            t1 = drive.trajectoryBuilder(drive.getPoseEstimate()).splineTo(new Vector2d( -54, 34.5), Math.toRadians(160))
                    .addTemporalMarker(.2,()->{drive.topClaw.setPosition(.1);})
                    .addTemporalMarker(1.2,()->{}).build();
            t2 = drive.trajectoryBuilder(new Pose2d(-28,32,75),-90)
                    .splineToLinearHeading(new Pose2d(14,36,Math.toRadians(0)),Math.toRadians(0)).build();
            t7 = drive.trajectoryBuilder(new Pose2d(14,36,Math.toRadians(180)),Math.toRadians(0))
                    .splineToLinearHeading(new Pose2d(-52,30,Math.toRadians(245)),Math.toRadians(-90)).build();
            t8 = drive.trajectoryBuilder(new Pose2d(-52, 30,Math.toRadians(65)),Math.toRadians(-90))
                    .splineToLinearHeading(new Pose2d(14, 36, Math.toRadians(0)), Math.toRadians(0)).build();
            t9 = drive.trajectoryBuilder(new Pose2d(14,36,Math.toRadians(0)),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(0,36),Math.toRadians(0)).build();
        } catch (Exception e) {
            e.printStackTrace();
        }
        /*telemetry.setData("Trajectories", "initialized");
        telemetry.setData("Acceleration",t3.acceleration(t3.duration()).toString());
        telemetry.setData("Duration",t3.duration());
        telemetry.setData("End",t3.end().toString());
        telemetry.setData("Start",t3.start().toString());
        telemetry.setData("Path",t3.getPath().toString());
        telemetry.setData("Velocity", t3.velocity(t3.duration()).toString());*/
        telemetry.setData("Acceleration", t1.acceleration(t1.duration()).toString());
        telemetry.setData("Duration", t1.duration());
        telemetry.setData("End", t1.end().toString());
        telemetry.setData("Start", t1.start().toString());
        telemetry.setData("Path", t1.getPath().toString());
        telemetry.setData("Velocity", t1.velocity(t1.duration()).toString());

        waitForStart();
        if (isStopRequested()) return;
        runtime.reset();
        drive.followTrajectory(t1);
        drive.topClaw.setPosition(.8);
        sleep(300);
        drive.followTrajectory(t2);
        drive.topClaw.setPosition(.1);
        drive.followTrajectory(t7);
        drive.topClaw.setPosition(.8);
        sleep(300);
        drive.followTrajectory(t8);
        drive.topClaw.setPosition(.1);
        drive.followTrajectory(t9);
        //drive.followTrajectory(t3);


    }
}
