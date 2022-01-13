package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.hermitsocialclub.util.MoveUtils.m;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.telecat.PersistantTelemetry;

@Autonomous(name = "Meet2Auto")
public class Meet2Auto extends LinearOpMode {

    private BaselineMecanumDrive drive;
    private PersistantTelemetry telemetry;

    Trajectory backUp;

    Pose2d blueStart =  new Pose2d(-38,63,m(-90));


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new PersistantTelemetry(super.telemetry);
        drive = new BaselineMecanumDrive(hardwareMap,telemetry);

        drive.setPoseEstimate(blueStart);

        backUp = drive.trajectoryBuilder(blueStart, -90)
                .splineTo(new Vector2d(-53.50, 53.50), Math.toRadians(135))
                .build();

        waitForStart();


            drive.followTrajectory(backUp);




    }
}
