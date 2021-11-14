package org.hermitsocialclub.opmodes.freightfrenzy;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.telecat.PersistantTelemetry;

import static org.hermitsocialclub.util.MoveUtils.m;

@Autonomous(name = "AutoBlue")
public class LinearOutlineAuto extends LinearOpMode {

    Trajectory longBoi; //Main drag between the carousel to the team element to the hub to the freight zone
    PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);
    FtcDashboard dashboard;

    BaselineMecanumDrive drive;


    //Robot Poses
    Pose2d startPose = new Pose2d(-58, 58, m(-35));
    Pose2d teamElementGrab = new Pose2d(-30, 46, 0);
    Pose2d dropFreight = new Pose2d(-12, 42, m(-90));
    Pose2d goToBarrier = new Pose2d(20, 42, 0);
    Pose2d pickUpFreight = new Pose2d(48, 48, m(45));

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new BaselineMecanumDrive(hardwareMap, telemetry);

        dashboard = FtcDashboard.getInstance();

        drive.setPoseEstimate(startPose);

        longBoi = drive.trajectoryBuilder(startPose, m(-45))//starts at the carousel
                .splineToSplineHeading(teamElementGrab, m(-10)) //Goes along team elements
                //.addDisplacementMarker(()->{/*TODO: Add code to grab the team element*/})
                //.splineToSplineHeading(dropFreight,m(-10)) //Goes to the shipping hub
                //.addDisplacementMarker(()->{/*TODO: Outtake the Freight*/})
                //.splineToSplineHeading(goToBarrier,0) //Turns while going to the barrier,
                // so you don't need to turn while crossing it
                //.splineToSplineHeading(pickUpFreight,m(25))//Goes towards the freight
                //.addDisplacementMarker(()->{/*TODO: Intake the Freight*/})
                .build();
        //TODO: Initialize trajectory grid for tele-op

        //TODO: Continually scan for the team element

        waitForStart();

        while (opModeIsActive()) {

            //TODO: Turn the Carousel


            drive.followTrajectory(longBoi);


        }
    }
}
