package org.hermitsocialclub.opmodes.freightfrenzy;

import static org.hermitsocialclub.util.MoveUtils.m;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import com.spartronics4915.lib.T265Camera;
import com.spartronics4915.lib.T265Helper;
import com.spartronics4915.lib.T265Localizer;
import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.hydra.vision.VisionPipeline;
import org.hermitsocialclub.hydra.vision.FirstFrameSemaphore;
import org.hermitsocialclub.localizers.T265LocalizerPro;
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
    static double secondInches = 11.5;
    static double thirdInches = 17.75;
    static int liftTimeout = 2000;

    //Duck Wheel
    MotorConfigurationType duckType;
    static double duckSpeed = .3;

    //Key Positions for Blue
    Pose2d blueStart = new Pose2d(-38,63,m(90));
    Vector2d blueCarouselPrep = new Vector2d(-48.5,48.5);
    Vector2d blueCarousel = new Vector2d(-53.5,53.5);
    Pose2d blueElementDrag = new Pose2d(-30,43,m(-90));
    Vector2d blueHub = new Vector2d(-12,44);
    Pose2d blueBarrier = new Pose2d(16,44,0);
    Pose2d blueWarehouse = new Pose2d(48,48,m(45));

    //Blue Trajectories
    Trajectory toCarouselBlue1;
    Trajectory toCarouselBlue2;
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
        drive.setLocalizer(new T265LocalizerPro(hardwareMap));

        duckType = drive.duck_wheel.getMotorType();

        liftType = drive.lift.getMotorType();

        //Blue Trajectory Initialization
         toCarouselBlue1 = drive.trajectoryBuilder(blueStart,m(-90))
                .back(15.5)
                .build();
         toCarouselBlue2 = drive.trajectoryBuilder(toCarouselBlue1.end()
                 .plus(new Pose2d(0,0,m(45))))
                 .lineTo(blueCarousel)
                .build();
         toHubBlue = drive.trajectoryBuilder(toCarouselBlue1.end(),m(-45))
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
                .addDisplacementMarker(() -> {drive.intake.setPower(
                        .85
                );})
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
            case RED:      /*{
                drive.setPoseEstimate(redStart);
                drive.followTrajectory(toCarouselRed);
                drive.duck_wheel.setVelocity(duckSpeed * duckType.getMaxRPM()/60
                        * duckType.getAchieveableMaxRPMFraction() * 2 * Math.PI,
                        AngleUnit.RADIANS);
                sleep(500);
                drive.duck_wheel.setVelocity(0);
                drive.followTrajectory(toHubRed);
                switch (code){
                    case -1:
                    case  1:
                        drive.outtakeArm.setPosition(.5); break;
                    case  2:{
                        ElapsedTime timer = new ElapsedTime();
                        drive.lift.setTargetPosition((int) (liftType.getTicksPerRev()
                        * secondInches / (2.5 * Math.PI)));
                        drive.lift.setVelocity(liftType.getAchieveableMaxRPMFraction()
                                * liftType.getMaxRPM() * 1/60 * Math.PI * 2
                                * liftSpeed);
                        timer.reset();
                        while (drive.lift.isBusy() && timer.milliseconds() < liftTimeout){
                            telemetry.setData("Lift Position:",
                                    drive.lift.getCurrentPosition());
                        }
                        drive.lift.setVelocity(0);
                        drive.outtakeArm.setPosition(.5);
                        break;
                    }
                    case  3:{
                        ElapsedTime timer = new ElapsedTime();
                        drive.lift.setTargetPosition((int) (liftType.getTicksPerRev()
                                * thirdInches / (2.5 * Math.PI)));
                        drive.lift.setVelocity(liftType.getAchieveableMaxRPMFraction()
                                * liftType.getMaxRPM() * 1/60 * Math.PI * 2
                                * liftSpeed);
                        timer.reset();
                        while (drive.lift.isBusy() && timer.milliseconds() < liftTimeout){
                            telemetry.setData("Lift Position:",
                                    drive.lift.getCurrentPosition());
                        }
                        drive.lift.setVelocity(0);
                        drive.outtakeArm.setPosition(.5);
                        break;
                    }
                }
                drive.followTrajectory(toWarehouseRed);
                sleep(500);
                drive.intake.setPower(0);
                break;
            }*/
            case BLUE: {
                drive.setPoseEstimate(blueStart);
                drive.followTrajectory(toCarouselBlue1);
//                drive.turn(m(45));
//                drive.followTrajectory(toCarouselBlue2);
//                drive.duck_wheel.setPower(duckSpeed);
//                sleep(500);
//                drive.duck_wheel.setVelocity(0);
                /*drive.followTrajectory(toHubBlue);
                switch (code){
                    case -1:
                    case  1:
                        drive.outtakeArm.setPosition(.5); break;
                    case  2:{
                        ElapsedTime timer = new ElapsedTime();
                        drive.lift.setTargetPosition((int) (liftType.getTicksPerRev()
                                * secondInches / (2.5 * Math.PI)));
                        drive.lift.setVelocity(liftType.getAchieveableMaxRPMFraction()
                                * liftType.getMaxRPM() * 1/60 * Math.PI * 2
                                * liftSpeed);
                        timer.reset();
                        while (drive.lift.isBusy() && timer.milliseconds() < liftTimeout){
                            telemetry.setData("Lift Position:",
                                    drive.lift.getCurrentPosition());
                        }
                        drive.lift.setVelocity(0);
                        drive.outtakeArm.setPosition(.5);
                        break;
                    }
                    case  3:{
                        ElapsedTime timer = new ElapsedTime();
                        drive.lift.setTargetPosition((int) (liftType.getTicksPerRev()
                                * thirdInches / (2.5 * Math.PI)));
                        drive.lift.setVelocity(liftType.getAchieveableMaxRPMFraction()
                                * liftType.getMaxRPM() * 1/60 * Math.PI * 2
                                * liftSpeed);
                        timer.reset();
                        while (drive.lift.isBusy() && timer.milliseconds() < liftTimeout){
                            telemetry.setData("Lift Position:",
                                    drive.lift.getCurrentPosition());
                        }
                        drive.lift.setVelocity(0);
                        drive.outtakeArm.setPosition(.5);
                        break;
                    }
                }
                drive.followTrajectory(toWarehouseBlue);
                sleep(500);
                drive.intake.setPower(0);*/
                break;
            }
        }
    }
    private int barCode() {
        semaphore.waitForFirstFrame();
        return barcodeDetect.getResult();
    }
}
