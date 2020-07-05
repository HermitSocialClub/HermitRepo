package org.firstinspires.ftc.teamcode.drive.opmode;

import android.content.Context;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfigManager;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.legacy.AutoUtils;
import org.firstinspires.ftc.teamcode.util.AssetsTrajectoryManager;
import org.firstinspires.ftc.teamcode.vision.SkystoneVuforiaEngine;
import org.hermitsocialclub.pandemicpanic.MecanumConfiguration;
import org.hermitsocialclub.pandemicpanic.telecat.PersistantTelemetry;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;
@Autonomous(name = "RR Auto Skystone")
public class SkystoneAutoAttempt extends LinearOpMode {
    Trajectory t3;
    Trajectory t5;
    Trajectory t2;
    Trajectory foundationApproach;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();
        PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);
        SkystoneVuforiaEngine vuforiaEngine = SkystoneVuforiaEngine.get(telemetry);
       // vuforiaEngine.init(hardwareMap);
        //telemetry.setData("vuforia intitialized?",vuforiaEngine.hasAlreadyBeenInit);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap,vuforiaEngine);
        TrajectoryGroupConfig tgc = new TrajectoryGroupConfig(drive.constraints.maxVel,drive.constraints.maxAccel,drive.constraints.maxAngVel,drive.constraints.maxAngAccel,15,10.75, TrajectoryGroupConfig.DriveType.MECANUM, DriveConstants.TRACK_WIDTH,DriveConstants.TRACK_WIDTH,1.0);
        //MecanumConfiguration robot = new MecanumConfiguration();
        //robot.init(hardwareMap);
        //Supplier<AtomicInteger> left1Encoder = () -> new AtomicInteger(drive.leftFront.getCurrentPosition());
        //Supplier<AtomicInteger> left2Encoder = () -> new AtomicInteger(drive.leftRear.getCurrentPosition());
        //Supplier<AtomicInteger> right1Encoder = () -> new AtomicInteger(drive.rightFront.getCurrentPosition());
        //Supplier<AtomicInteger> right2Encoder = () -> new AtomicInteger(drive.rightRear.getCurrentPosition());
        //AutoUtils util =
          //      new AutoUtils(telemetry,robot,this::opModeIsActive, runtime, left1Encoder,left2Encoder,right1Encoder,right2Encoder);
        try {
            foundationApproach = drive.trajectoryBuilder(new Pose2d(38,64)).forward(33.0).build();
            t2 = drive.trajectoryBuilder(new Pose2d(33.0,0),0).splineToLinearHeading(new Pose2d(15.0,
                    9.0),Math.toRadians(90)).build();/*AssetsTrajectoryManager.loadConfig("Foundation Pull").
                    toTrajectory(tgc);*/
            t3 = drive.trajectoryBuilder(drive.getPoseEstimate(),drive.getExternalHeading())
                    .splineToLinearHeading(
                    new Pose2d(-64,-15),Math.toRadians(90)).build();
            t5 = AssetsTrajectoryManager.loadConfig("Blocks to Foundation").
                    toTrajectory(tgc);

        } catch (Exception e) {
            e.printStackTrace();
        }
        telemetry.setData("Trajectories", "initialized");
        telemetry.setData("Acceleration",t2.acceleration(t2.duration()).toString());
        telemetry.setData("Duration",t2.duration());
        telemetry.setData("End",t2.end().toString());
        telemetry.setData("Start",t2.start().toString());
        telemetry.setData("Path",t2.getPath().toString());
        telemetry.setData("Velocity", t2.velocity(t2.duration()).toString());
        telemetry.setData("Trajectories", "initialized");
        telemetry.setData("Acceleration",t3.acceleration(t3.duration()).toString());
        telemetry.setData("Duration",t3.duration());
        telemetry.setData("End",t3.end().toString());
        telemetry.setData("Start",t3.start().toString());
        telemetry.setData("Path",t3.getPath().toString());
        telemetry.setData("Velocity", t3.velocity(t3.duration()).toString());
        waitForStart();
        if (isStopRequested()) return;

        /*foundationApproach = drive.trajectoryBuilder(new Pose2d(38.0, 63.0, -1.5707963267948966))
                .splineToConstantHeading(new Pose2d(38, 31)).build();
        t2 = drive.trajectoryBuilder(new Pose2d(38,31,-1.5707963267948966),
                4.71238898038469).
                splineToLinearHeading(new Pose2d(44,48,0),0)
                .build();
        t3 = drive.trajectoryBuilder(new Pose2d(44,48,1.5707963267948966)
        ,0).splineToLinearHeading(new Pose2d(-20,33,0),
                -1.5707963267948966).build();
        t5 = drive.trajectoryBuilder(new Pose2d(-44,33,1.0471975511965976),
                -1.5707963267948966)
                .splineToLinearHeading(new Pose2d(40,38,-1.5707963267948966),0)
                .build();*/
        //util.Linear(1,.3,-7);
        drive.followTrajectory(foundationApproach);
        sleep(200);
        //util.Linear(-1,.3,8);
        drive.followTrajectory(t2);
        //util.Linear(1,.3,-3);
        //util.Linear(-1,.3,4);
        drive.followTrajectory(t3);
        //util.move_With_Color(AutoUtils.Color.YELLOW, AutoUtils.Direction.RIGHT,.25,26,25000, AutoUtils.Side.BLUE);
        //robot.topClaw.setPosition(1);
        //util.Linear(1,.3,-3);
      //  drive.followTrajectory(t5);
        //util.Linear(1,.3,-4);
        /*Trajectory t6 = drive.trajectoryBuilder(new Pose2d(40,38,-1.0471975511965976),0)
                .splineToConstantHeading(new Pose2d(44,38,-1.0471975511965976)).build();
        //robot.topClaw.setPosition(.2);
        drive.followTrajectory(t6);*/

    }
}
