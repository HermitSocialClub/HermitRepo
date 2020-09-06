package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AssetsTrajectoryManager;
import org.firstinspires.ftc.teamcode.vision.SkystoneVuforiaEngine;
import org.hermitsocialclub.telecat.PersistantTelemetry;

@Autonomous(name = "RR Auto Skystone2")
public class SkystoneAutoAttempt2 extends LinearOpMode {
    Trajectory t3;
    Trajectory t5;
    Trajectory t2;
    Trajectory foundationApproach;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();
        PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);
        SkystoneVuforiaEngine vuforiaEngine = SkystoneVuforiaEngine.get(telemetry);
       SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap,vuforiaEngine);
        TrajectoryGroupConfig tgc = new TrajectoryGroupConfig(drive.constraints.maxVel,drive.constraints.maxAccel,drive.constraints.maxAngVel,drive.constraints.maxAngAccel,15,10.75, TrajectoryGroupConfig.DriveType.MECANUM, DriveConstants.TRACK_WIDTH,DriveConstants.TRACK_WIDTH,1.0);
        try {
            foundationApproach = drive.trajectoryBuilder(new Pose2d()).forward(33.0).build();
            t2 = drive.trajectoryBuilder(new Pose2d()).splineToLinearHeading(new Pose2d(6.0,-17.0),Math.toRadians(90)).build();/*AssetsTrajectoryManager.loadConfig("Foundation Pull").
                    toTrajectory(tgc);*/
            t3 = AssetsTrajectoryManager.loadConfig("Founda" +
                    "tion Pull to Blocks").
                    toTrajectory(tgc);
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
        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectory(foundationApproach);
        drive.followTrajectory(t2);
        drive.followTrajectory(t3);
        drive.followTrajectory(t5);


    }
}
