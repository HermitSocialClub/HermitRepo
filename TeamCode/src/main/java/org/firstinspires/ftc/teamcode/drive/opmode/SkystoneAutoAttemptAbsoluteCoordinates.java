package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AssetsTrajectoryManager;
import org.firstinspires.ftc.teamcode.vision.SkystoneVuforiaEngine;
import org.hermitsocialclub.pandemicpanic.telecat.PersistantTelemetry;

@Autonomous(name = "RR Auto Skystone AC")
public class SkystoneAutoAttemptAbsoluteCoordinates extends LinearOpMode {
    Trajectory t3;
    Trajectory t5;
    Trajectory t2;
    Trajectory t1;
    Trajectory t6;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();
        PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);
        SkystoneVuforiaEngine vuforiaEngine = SkystoneVuforiaEngine.get(telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap,vuforiaEngine);
        TrajectoryGroupConfig tgc = new TrajectoryGroupConfig(drive.constraints.maxVel,drive.constraints.maxAccel,drive.constraints.maxAngVel,drive.constraints.maxAngAccel,15,10.75, TrajectoryGroupConfig.DriveType.MECANUM, DriveConstants.TRACK_WIDTH,DriveConstants.TRACK_WIDTH,1.0);
        drive.setPoseEstimate(new Pose2d(38,64,Math.toRadians(-90)));
        try {
            t1 = drive.trajectoryBuilder(drive.getPoseEstimate()).splineTo(new Pose2d( 38, 33, Math.toRadians(-90)))
                    .splineTo(new Pose2d(28,44,Math.toRadians(0))).addTemporalMarker(1,()->drive.LinearSync(-.9,-11)).build();
            t6 = drive.trajectoryBuilder(new Pose2d()).splineTo(new Pose2d(28,44,Math.toRadians(0))).build();
            t2 = drive.trajectoryBuilder(t1.end()).splineToLinearHeading(new Pose2d(-44,32,Math.toRadians(-90)),Math.toRadians(-90)).build();
            /*AssetsTrajectoryManager.loadConfig("Foundation Pull").
                    toTrajectory(tgc);*/
            t3 = drive.trajectoryBuilder(new Pose2d())
                    .splineToLinearHeading(
                    new Pose2d(-64,-15),Math.toRadians(90)).build();
            t5 = AssetsTrajectoryManager.loadConfig("Blocks to Foundation").
                    toTrajectory(tgc);

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
        drive.followTrajectoryAsync(t1);
        while (drive.isBusy()){
            drive.update();
        }
        //drive.LinearSync(-.9,-18);
        //drive.waitForIdle();
        //drive.followTrajectory(t6);
        //sleep(2000);
        drive.followTrajectory(t2);
        //drive.followTrajectory(t3);


    }
}
