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
    Trajectory toBlueHub;

    Pose2d blueStart =  new Pose2d(-38,63,m(90));
    Pose2d blueCarousel = new Pose2d(-12,44,m(90));


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new PersistantTelemetry(super.telemetry);
        drive = new BaselineMecanumDrive(hardwareMap,telemetry);

        drive.setPoseEstimate(blueStart);

        backUp = drive.trajectoryBuilder(blueStart, -90)
                .splineToSplineHeading(new Pose2d(-59.1360, 59.1360,m(135)), m(135))
                .build();

        toBlueHub = drive.trajectoryBuilder(backUp.end())
                .splineToLinearHeading(blueCarousel,m(-90))
                .build();

        waitForStart();


            drive.followTrajectory(backUp);
            drive.duck_wheel.setPower(.3);
            sleep(300);
            drive.duck_wheel.setPower(0);
            drive.followTrajectory(toBlueHub);




    }
}
