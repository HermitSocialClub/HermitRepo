package org.hermitsocialclub.opmodes.freightfrenzy;

import static org.hermitsocialclub.util.MoveUtils.m;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.hydra.vision.VisionPipeline;
import org.hermitsocialclub.hydra.vision.FirstFrameSemaphore;
import org.hermitsocialclub.telecat.PersistantTelemetry;
import org.hermitsocialclub.tomato.BarcodeDetect;

//m is Math.toRadians, I'm just too lazy to type that out
@Autonomous(name = "Meet1Auto")
public class Meet1Auto extends LinearOpMode {

    //Basic Utility
    BaselineMecanumDrive drive;
    PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);

    //Linear Lift
    MotorConfigurationType liftType;
    static double liftSpeed = .45;

    //Duck Wheel
    MotorConfigurationType duckType;
    static double duckSpeed;

    //Key Positions for Blue
    Pose2d blueStart = new Pose2d(-38,63,m(-90));
    Pose2d blueCarouselPrep = new Pose2d(-48.5,48.5,m(-45));
    Vector2d blueCarousel = new Vector2d(-53.5,53.5);
    Pose2d blueElementDrag = new Pose2d(-30,43,m(-90));
    Vector2d blueHub = new Vector2d(-12,44);
    Pose2d blueBarrier = new Pose2d(16,44,0);
    Pose2d blueWarehouse = new Pose2d(48,48,m(45));

    //Blue Trajectories
    Trajectory toCarouselBlue;
    Trajectory toHubBlue;
    Trajectory toWarehouseBlue;

    //Key Positions Red
    Pose2d redStart = new Pose2d(-38,-63,m(90));
    Pose2d redCarouselPrep = new Pose2d(-48.5,-48.5,m(45));
    Vector2d redCarousel = new Vector2d(-53.5,-53.5);
    Pose2d redElementDrag = new Pose2d(-30,-43,m(90));
    Vector2d redHub = new Vector2d(-12,-44);
    Pose2d redBarrier = new Pose2d(16,-44,0);
    Pose2d redWarehouse = new Pose2d(48,-48,m(-45));


    //Red Trajectories
    Trajectory toCarouselRed;
    Trajectory toHubRed;
    Trajectory toWarehouseRed;

    //Switching Between Red and Blue Sides
    public static enum side {
        RED, BLUE

    }
    side colorSide = side.BLUE;

    //Vision
    private VisionPipeline visionPipeline;
    private BarcodeDetect barcodeDetect;
    private FirstFrameSemaphore semaphore;
    private int code = -1;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new BaselineMecanumDrive(hardwareMap, telemetry);

        duckType = drive.duck_wheel.getMotorType();

        liftType = drive.lift.getMotorType();

        //Blue Trajectory Initialization
         toCarouselBlue = drive.trajectoryBuilder(blueStart,m(-90))
                .splineToSplineHeading(blueCarouselPrep,m(135))
                .splineTo(blueCarousel,m(-45))
                .build();
         toHubBlue = drive.trajectoryBuilder(toCarouselBlue.end(),m(-45))
                .splineToSplineHeading(blueElementDrag,m(0))
                .splineToConstantHeading(blueHub,m(0))
                .build();
         toWarehouseBlue = drive.trajectoryBuilder(toHubBlue.end(),0)
                .splineToSplineHeading(blueBarrier,0)
                .splineToSplineHeading(blueWarehouse,m(20))
                .build();

        //Red Trajectory Initialization
        toCarouselRed = drive.trajectoryBuilder(redStart,m(-90))
                .splineToSplineHeading(redCarouselPrep,m(135))
                .splineTo(redCarousel,m(-45))
                .build();
        toHubRed = drive.trajectoryBuilder(toCarouselRed.end(),m(-45))
                .splineToSplineHeading(redElementDrag,m(0))
                .splineToConstantHeading(redHub,m(0))
                .build();
        toWarehouseRed = drive.trajectoryBuilder(toHubRed.end(),0)
                .splineToSplineHeading(redBarrier,0)
                .splineToSplineHeading(redWarehouse,m(20))
                .build();

        telemetry.setData("Trajectories:", "Initialized");

        // init vision
        barcodeDetect = new BarcodeDetect(true);
        semaphore = new FirstFrameSemaphore();
        visionPipeline = new VisionPipeline(hardwareMap, telemetry, barcodeDetect, semaphore);

        while (!isStarted()){
            code = barCode();
            if(gamepad1.a || gamepad2.a){
                if (colorSide == side.RED){
                    colorSide = side.BLUE;
                } else colorSide = side.RED;
            }
            telemetry.setData("Side",colorSide.toString());
        }

        waitForStart();

        switch (colorSide){
            case RED: {
                drive.followTrajectory(toCarouselRed);
                drive.duck_wheel.setVelocity(duckSpeed * duckType.getMaxRPM()/60
                        * duckType.getAchieveableMaxRPMFraction() * 2 * Math.PI,
                        AngleUnit.RADIANS);
                sleep(500);
                drive.duck_wheel.setVelocity(0);
                drive.followTrajectory(toHubRed);
                switch (code){
                    case -1: drive.outtakeArm.setPosition(.5);
                }
                break;
            }
        }
    }
    private int barCode() {
        semaphore.waitForFirstFrame();
        return barcodeDetect.getResult();
    }
}
