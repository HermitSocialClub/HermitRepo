package org.hermitsocialclub.opmodes.freightfrenzy;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.hydra.vision.VisionSemaphore;
import org.hermitsocialclub.telecat.PersistantTelemetry;

import static org.hermitsocialclub.util.MoveUtils.m;

@Autonomous(name = "MainAutoBlue")
public class BasicallyJustAnOutlineAuto extends LinearOpMode {

    Trajectory longBoi; //Main drag between the carousel to the team element to the hub to the freight zone
    Trajectory shortBoi;
    Trajectory strafe;
    PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);
    FtcDashboard dashboard;
    Trajectory shortStrafe;
    Trajectory longStrafe;

    private final ElapsedTime time = new ElapsedTime();


    private final int code = -1;


    BaselineMecanumDrive drive;


    //Robot Poses
    Pose2d startPose = new Pose2d(-48, 63, m(-90));
    Pose2d teamElementGrab = new Pose2d(-30, 46, 0);
    Pose2d dropFreight = new Pose2d(-12, 42, m(-90));
    Pose2d goToBarrier = new Pose2d(20, 42, 0);
    Pose2d pickUpFreight = new Pose2d(48, 48, m(45));
    private VisionSemaphore semaphore;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new BaselineMecanumDrive(hardwareMap, telemetry);

        dashboard = FtcDashboard.getInstance();

        drive.setPoseEstimate(startPose);

        while (!isStarted()) {
            telemetry.setData("Press A to strafe left and B to strafe right", "");
            if (gamepad1.a) {
                shortStrafe = drive.trajectoryBuilder(startPose, 0)
                        .back(72)
                        .build();
            }
            if (gamepad1.b) {
                shortStrafe = drive.trajectoryBuilder(startPose, 0)
                        .forward(72)
                        .build();
            }
        }
        longStrafe = drive.trajectoryBuilder(shortStrafe.end(), 0)
                .splineToConstantHeading(new Vector2d(-12, 42), 0)
                .build();
        longBoi = drive.trajectoryBuilder(longStrafe.end(), m(-45))//starts at the carousel
                .splineToSplineHeading(goToBarrier, m(0)) //Goes along team elements
                //.addDisplacementMarker(()->{/*TODO: Add code to grab the team element*/})
                //.splineToSplineHeading(dropFreight,m(-10)) //Goes to the shipping hub
                //.addDisplacementMarker(()->{/*TODO: Outtake the Freight*/})
                //.splineToSplineHeading(goToBarrier,0) //Turns while going to the barrier,
                // so you don't need to turn while crossing it
                .splineToSplineHeading(pickUpFreight, m(25))//Goes towards the freight
                //.addDisplacementMarker(()->{/*TODO: Intake the Freight*/})
                .build();
        shortBoi = drive.trajectoryBuilder(startPose, 0)
                .strafeRight(1)
                .build();
        strafe = drive.trajectoryBuilder(shortBoi.end(), 0)
                .strafeLeft(108)
                .build();

        //barcodeDetect = new BarcodeDetect(true);
        //semaphore = new VisionSemaphore();
        //visionPipeline = new VisionPipeline(hardwareMap, telemetry, barcodeDetect, semaphore);
        //TODO: Initialize trajectory grid for tele-op
        waitForStart();
        drive.lift.setPower(.4);
        sleep(400);
        drive.lift.setPower(0.000005);
        drive.followTrajectory(shortStrafe);
        /*drive.duck_wheel.setPower(-.3);
        sleep(400);
        drive.duck_wheel.setPower(0);
        drive.followTrajectory(longStrafe);
        drive.intake.setPower(.5);
        sleep(500);
        drive.intake.setPower(0);
        drive.followTrajectory(longBoi);*/
    }
}
