package org.hermitsocialclub.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.hydra.vision.VisionPipeline;
import org.hermitsocialclub.hydra.vision.VisionSemaphore;
import org.hermitsocialclub.telecat.PersistantTelemetry;
import org.hermitsocialclub.tomato.BarcodeDetect;

//m() is just Math.toRadians, I'm just lazy

@Autonomous(name = "Meet0Linear")
public class Meet0Linear extends LinearOpMode {

    Trajectory longBoi; //Main drag between the carousel to the team element to the hub to the freight zone
    Trajectory shortBoi;
    Trajectory strafe;
    PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);
    FtcDashboard dashboard;

    private VisionPipeline visionPipeline;
    private BarcodeDetect barcodeDetect;

    private ElapsedTime time = new ElapsedTime();




    private int code = -1;


    BaselineMecanumDrive drive;


    //Robot Poses
    Pose2d startPose = new Pose2d(-48,-63,m(-90));
    Pose2d teamElementGrab = new Pose2d(-30,46,0);
    Pose2d dropFreight = new Pose2d(-12,42,m(-90));
    Pose2d goToBarrier = new Pose2d(20,42,0);
    Pose2d pickUpFreight = new Pose2d(48,48,m(45));
    private VisionSemaphore semaphore;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new BaselineMecanumDrive(hardwareMap,telemetry);

        dashboard = FtcDashboard.getInstance();

        drive.setPoseEstimate(startPose);

        longBoi = drive.trajectoryBuilder(startPose,m(-45))//starts at the carousel
                .splineToSplineHeading(teamElementGrab,m(-10)) //Goes along team elements
                //.addDisplacementMarker(()->{/*TODO: Add code to grab the team element*/})
                //.splineToSplineHeading(dropFreight,m(-10)) //Goes to the shipping hub
                //.addDisplacementMarker(()->{/*TODO: Outtake the Freight*/})
                //.splineToSplineHeading(goToBarrier,0) //Turns while going to the barrier,
                // so you don't need to turn while crossing it
                //.splineToSplineHeading(pickUpFreight,m(25))//Goes towards the freight
                //.addDisplacementMarker(()->{/*TODO: Intake the Freight*/})
                .build();
        shortBoi = drive.trajectoryBuilder(startPose,0)
                .lineToConstantHeading(new Vector2d(-53,-63))
                .build();
        strafe = drive.trajectoryBuilder(shortBoi.end(),0)
                .lineToConstantHeading(new Vector2d(53,-63))
                .build();

        //barcodeDetect = new BarcodeDetect(true);
        //semaphore = new VisionSemaphore();
        //visionPipeline = new VisionPipeline(hardwareMap, telemetry, barcodeDetect, semaphore);
        //TODO: Initialize trajectory grid for tele-op
        //TODO: Turn the Carousel
        waitForStart();
        time.reset();
        drive.lift.setPower(.4);
        sleep(500);
        drive.lift.setPower(.000005);
        drive.followTrajectory(shortBoi);
        drive.duck_wheel.setPower(.5);
        sleep(500);
        drive.duck_wheel.setPower(0);
        drive.followTrajectory(strafe);
    }

    private double m(double heading){
        return Math.toRadians(heading);
    }

    private int barCode(){
        semaphore.waitForFrame();
        return barcodeDetect.getResult();
    }

}
