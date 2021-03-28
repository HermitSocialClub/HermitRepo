package org.hermitsocialclub.pandemicpanic;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.opmode.BaselineMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;
import org.hermitsocialclub.hydra.vision.StaccDetecc;
import org.hermitsocialclub.hydra.vision.VisionPipeline;
import org.hermitsocialclub.hydra.vision.VisionSemaphore;
import org.hermitsocialclub.telecat.PersistantTelemetry;
import org.opencv.core.Mat;

@Autonomous(name = "Scrimmage5Auto")
public class Scrimmage5Auto extends LinearOpMode {

    private static final double POWER_SHOT_PERCENT = 0.575;
    private static final double SPEED_PERCENT = 0.665;
    private static final double COLLECT_LAUNCH_PERCENT = 0.645;
    private static final double COLLECT_LATE_LAUNCH_PERCENT = .665;
    private PersistantTelemetry telemetry;
    private MotorConfigurationType goBildaOutTake;
    private BaselineMecanumDrive drive;
    private double launchLineSpeed;
    private double collectLaunchSpeed;
    private double collectLateLaunchSpeed;
    private double powerShotSpeed;
    private int ringStack = 0;
    private StaccDetecc stackDetector;
    private VisionSemaphore semaphore;
    private VisionPipeline visionPipeline;
    private ElapsedTime time;
    private enum Ambition{
        BASE_MODE, POWERSHOT_MODE,TURN_POWER_SHOT,CONSTANT_POWER_SHOT, BROKEN
    }
    private Ambition ambition = Ambition.BROKEN;

    @Override
    public void runOpMode() throws InterruptedException {
        // Init stuff
        this.telemetry = new PersistantTelemetry(super.telemetry);
        this.goBildaOutTake = MotorConfigurationType.getMotorType(GoBILDA5202Series.class);
        this.launchLineSpeed = ((SPEED_PERCENT * 2 * Math.PI * goBildaOutTake.getMaxRPM() *
                goBildaOutTake.getAchieveableMaxRPMFraction()) / 60);
        this.collectLaunchSpeed = ((COLLECT_LAUNCH_PERCENT * 2 * Math.PI * goBildaOutTake.getMaxRPM() *
                goBildaOutTake.getAchieveableMaxRPMFraction()) / 60);
        this.collectLateLaunchSpeed = ((COLLECT_LAUNCH_PERCENT * 2 * Math.PI * goBildaOutTake.getMaxRPM() *
                goBildaOutTake.getAchieveableMaxRPMFraction()) / 60);
        this.powerShotSpeed = ((POWER_SHOT_PERCENT * 2 * Math.PI * goBildaOutTake.getMaxRPM() *
                goBildaOutTake.getAchieveableMaxRPMFraction()) / 60);
        this.drive = new BaselineMecanumDrive(hardwareMap, telemetry);
        this.drive.setPoseEstimate(new Pose2d(-63, -15.5, 0));

        this.stackDetector = new StaccDetecc();
        this.semaphore = new VisionSemaphore();
        this.visionPipeline = new VisionPipeline(hardwareMap, telemetry, stackDetector, semaphore);
        this.time = new ElapsedTime();
        this.time.reset();

        drive.wobbleGrab.setPower(1);
        drive.wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Find which Zone the ring stack is in
        while (time.seconds() < 4 && !isStarted() && !isStopRequested()) {
            ringStack = getZoneFromOpenCV();
            telemetry.setDebug("detected", ringStack);
        }
        telemetry.setDebug("done", "viewing");
        telemetry.setDebug("final position", ringStack);
        drive.wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Build trajectories


        Trajectory constantLaunchSpline = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(new Vector2d(-14,12.50),0)
                .build();
        Trajectory constantLaunchSpline2 = drive.trajectoryBuilder(constantLaunchSpline.end())
                .splineToConstantHeading(new Vector2d(-3,-24.50),0)
                .addSpatialMarker(new Vector2d(0,-2.50), () -> launchRing(1,powerShotSpeed))
                .addSpatialMarker(new Vector2d(0,-14.50), () -> launchRing(1,powerShotSpeed))
                .addSpatialMarker(new Vector2d(0,-20.50), () -> launchRing(1,powerShotSpeed))
                .build();
        Trajectory turnLaunchSpline = drive
                .trajectoryBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(() -> drive.wobbleGrab.setPower(1))
                .splineToLinearHeading(new Pose2d(-3, -42,Math.toRadians(12.5)), 60)
                .addDisplacementMarker(() -> {
                    drive.outtake.setVelocity(launchLineSpeed, AngleUnit.RADIANS);
                    while (opModeIsActive() && Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) - launchLineSpeed) > Math.pow(10, -1)) {
                        telemetry.setDebug("Outtake Velocity", drive.outtake.getVelocity(AngleUnit.RADIANS));
                        telemetry.setDebug("Ticks Outtake Velocity", drive.outtake.getVelocity(AngleUnit.DEGREES));
                        telemetry.setDebug("Ticks Outtake Position", drive.outtake.getCurrentPosition());
                    }
                    int ringsFired = 0;
                    while (opModeIsActive() && ringsFired < 1) {
                        telemetry.setDebug("ringsFired", ringsFired);
                        drive.kicker.setPosition(.7);
                        sleep(200);
                        drive.kicker.setPosition(.3);
                        while (Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) - launchLineSpeed) > Math.pow(10, -1)) {
                            telemetry.setDebug("Outtake Velocity", drive.outtake.getVelocity(AngleUnit.RADIANS));
                        }
                        ringsFired++;
                    }
                })
                .build();
        Trajectory powerLaunchSpline = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(-34,-2,0),0)
                .addDisplacementMarker(() -> {
                    drive.outtake.setVelocity(launchLineSpeed, AngleUnit.RADIANS);
                    while (opModeIsActive() && Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) - launchLineSpeed) > Math.pow(10, -1)) {
                        telemetry.setDebug("Outtake Velocity", drive.outtake.getVelocity(AngleUnit.RADIANS));
                        telemetry.setDebug("Ticks Outtake Velocity", drive.outtake.getVelocity(AngleUnit.DEGREES));
                        telemetry.setDebug("Ticks Outtake Position", drive.outtake.getCurrentPosition());
                    }
                    int ringsFired = 0;
                    while (opModeIsActive() && ringsFired < 1) {
                        telemetry.setDebug("ringsFired", ringsFired);
                        drive.kicker.setPosition(.7);
                        sleep(200);
                        drive.kicker.setPosition(.3);
                        while (Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) - launchLineSpeed) > Math.pow(10, -1)) {
                            telemetry.setDebug("Outtake Velocity", drive.outtake.getVelocity(AngleUnit.RADIANS));
                        }
                        ringsFired++;
                    }
                })
                .build();
        Trajectory secondPowerShot = drive.trajectoryBuilder(powerLaunchSpline.end())
                .lineToConstantHeading(new Vector2d(-34,-14.50))
                .addDisplacementMarker(() -> {
                    while (opModeIsActive() && Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) - launchLineSpeed) > Math.pow(10, -1)) {
                        telemetry.setDebug("Outtake Velocity", drive.outtake.getVelocity(AngleUnit.RADIANS));
                        telemetry.setDebug("Ticks Outtake Velocity", drive.outtake.getVelocity(AngleUnit.DEGREES));
                        telemetry.setDebug("Ticks Outtake Position", drive.outtake.getCurrentPosition());
                    }
                    int ringsFired = 0;
                    while (opModeIsActive() && ringsFired < 1) {
                        telemetry.setDebug("ringsFired", ringsFired);
                        drive.kicker.setPosition(.7);
                        sleep(200);
                        drive.kicker.setPosition(.3);
                        while (Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) - launchLineSpeed) > Math.pow(10, -1)) {
                            telemetry.setDebug("Outtake Velocity", drive.outtake.getVelocity(AngleUnit.RADIANS));
                        }
                        ringsFired++;
                    }
                })
                .build();
        Trajectory thirdPowerShot = drive.trajectoryBuilder(secondPowerShot.end())
                .lineToConstantHeading(new Vector2d(-34,-20.50))
                .addDisplacementMarker(() -> {
                    while (opModeIsActive() && Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) - launchLineSpeed) > Math.pow(10, -1)) {
                        telemetry.setDebug("Outtake Velocity", drive.outtake.getVelocity(AngleUnit.RADIANS));
                        telemetry.setDebug("Ticks Outtake Velocity", drive.outtake.getVelocity(AngleUnit.DEGREES));
                        telemetry.setDebug("Ticks Outtake Position", drive.outtake.getCurrentPosition());
                    }
                    int ringsFired = 0;
                    while (opModeIsActive() && ringsFired < 1) {
                        telemetry.setDebug("ringsFired", ringsFired);
                        drive.kicker.setPosition(.7);
                        sleep(200);
                        drive.kicker.setPosition(.3);
                        while (Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) - launchLineSpeed) > Math.pow(10, -1)) {
                            telemetry.setDebug("Outtake Velocity", drive.outtake.getVelocity(AngleUnit.RADIANS));
                        }
                        ringsFired++;
                    }
                })
                .build();
        Trajectory fourSplinePower = drive.trajectoryBuilder(thirdPowerShot.end())
                .splineToLinearHeading(new Pose2d(50,-36,270),0)
                .build();
        Trajectory launchSpline = drive
                .trajectoryBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(() -> drive.wobbleGrab.setPower(1))
                .splineToConstantHeading(new Vector2d(-3, -41), Math.toRadians(-80))
                .build();
        Trajectory oneSpline = drive
                .trajectoryBuilder(launchSpline.end())
                .forward(12)
                .build();
        Trajectory fourCollect = drive.trajectoryBuilder(launchSpline.end())
                .addDisplacementMarker(()->{drive.intake.setPower(-.7);})
                .back(16)
                .build();
        Trajectory fourCollect2 = drive.trajectoryBuilder(fourCollect.end())
                .addDisplacementMarker(()-> drive.intake.setPower(.9))
                .back(4,new MecanumConstraints(
                        new DriveConstraints(5, DriveConstants.MAX_ACCEL,0.0,
                                DriveConstants.MAX_ANG_VELO,DriveConstants.MAX_ANG_ACCEL,0.0),
                        DriveConstants.TRACK_WIDTH))
                .addDisplacementMarker(()-> launchRing(3,collectLaunchSpeed)
                    )
                .back(5,new MecanumConstraints(
                        new DriveConstraints(6.5, DriveConstants.MAX_ACCEL,0.0,
                                DriveConstants.MAX_ANG_VELO,DriveConstants.MAX_ANG_ACCEL,0.0),
                        DriveConstants.TRACK_WIDTH))
                .addDisplacementMarker(()->
                        launchRing(3,collectLaunchSpeed)
                )
                .build();
        Trajectory fourSpline = drive
                .trajectoryBuilder(launchSpline.end())
                .splineToConstantHeading(new Vector2d(38, -58), 0)
                .build();

        Trajectory noSquare = drive
                .trajectoryBuilder(launchSpline.end().plus(new Pose2d(0, 0, Math.toRadians(-60))))
                .forward(5.5)
                .build();
        Trajectory noRingDisengage = drive
                .trajectoryBuilder(noSquare.end())
                .back(9)
                .build();

        Trajectory noRings = drive
                .trajectoryBuilder(noRingDisengage.end(),Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(-30.50, -42.50,Math.toRadians(180)), Math.toRadians(180))
                .addDisplacementMarker(() -> drive.wobbleGrab.setPower(1))
                .build();
        Trajectory noRingsDrop = drive
                .trajectoryBuilder(noRings.end(), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(8.50, -56.50, Math.toRadians(-30)), 0)
                .build();
        Trajectory oneRingDisengage = drive.trajectoryBuilder(oneSpline.end())
                .back(9)
                .build();
        Trajectory oneRingPickUp = drive.trajectoryBuilder(oneRingDisengage.end())
                .splineToLinearHeading(new Pose2d(-36.50,-46.50,Math.toRadians(180)),Math.toRadians(180))
                .build();
        Trajectory oneRingSecondDrop = drive.trajectoryBuilder(noRings.end())
                .splineToLinearHeading(new Pose2d(12,-36.50,0),0)
                .build();
        Trajectory fourRingSecondDrop = drive.trajectoryBuilder(noRings.end())
                .splineToLinearHeading(new Pose2d(36.50,-58,0),0)
                .addDisplacementMarker(() -> drive.wobbleGrab.setPower(-1))
                .build();
        Trajectory fourBack = drive
                .trajectoryBuilder(fourSpline.end())
                .back(9)
                .build();
        Trajectory fourBack2 = drive
                .trajectoryBuilder(fourRingSecondDrop.end())
                .back(20)
                .build();
        Trajectory oneRingDisengage2 = drive.trajectoryBuilder(oneRingSecondDrop.end())
                .back(20)
                .build();
        Trajectory fourRingPickUp = drive.trajectoryBuilder(fourBack.end(),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-36.50,-46.50,Math.toRadians(180)),Math.toRadians(180))
                .build();

        Trajectory failure1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(()->drive.wobbleGrab.setPower(1))
                .forward(62.5)
                .build();

        Trajectory failureZero0 = drive.trajectoryBuilder(failure1.end())
                .forward(5)
                .build();
        Trajectory failureZero = drive.trajectoryBuilder(failureZero0.end().plus(new Pose2d(0,0,Math.toRadians(120))))
                .forward(4)
                .build();
        Trajectory failureOne = drive.trajectoryBuilder(failure1.end())
                .forward(18)
                .build();
        Trajectory failureFour = drive.trajectoryBuilder(failure1.end())
                .forward(48)
                .build();
        Trajectory failureFourBack = drive.trajectoryBuilder(failureFour.end().plus(new Pose2d(0,0,Math.toRadians(180))))
                .forward(36)
                .build();
        telemetry.setDebug("Achievable RPM Fraction", goBildaOutTake.getAchieveableMaxRPMFraction());
        telemetry.setDebug("Achievable Ticks Per Second", goBildaOutTake.getAchieveableMaxTicksPerSecond());
        telemetry.setDebug("hub velocity params", goBildaOutTake.getHubVelocityParams());
        telemetry.setDebug("velocity params", goBildaOutTake.hasExpansionHubVelocityParams());
        telemetry.setDebug("outtakeVelocity", launchLineSpeed);

        waitForStart();

        switch (ambition){
            case BROKEN:{
                drive.followTrajectory(failure1);
                launchRing(3,launchLineSpeed);
                if(ringStack == 1) {
                    drive.followTrajectory(failureOne);
                    drive.turn(Math.toRadians(180));
                    waitForDrive();
                    drive.wobbleGrab.setPower(-1);
                    sleep(600);
                }else if(ringStack == 0){
                    drive.followTrajectory(failureZero0);
                    drive.turn(Math.toRadians(120));
                    waitForDrive();
                    drive.wobbleGrab.setPower(-1);
                    sleep(600);
                    drive.followTrajectory(failureZero);
                }else if (ringStack == 4){
                    drive.followTrajectory(failureFour);
                    drive.turn(Math.toRadians(120));
                    waitForDrive();
                    drive.liftWobble(-125,.35,AngleUnit.DEGREES,1500);
                    drive.wobbleGrab.setPower(-1);
                    sleep(600);
                    drive.turn(Math.toRadians(60));
                    drive.followTrajectory(failureFourBack);
                }
                break;
            }
            case CONSTANT_POWER_SHOT:{
                drive.followTrajectory(constantLaunchSpline);
                drive.outtake.setVelocity(powerShotSpeed);
                while (opModeIsActive() && Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) - powerShotSpeed) > Math.pow(10, -1)) {
                    telemetry.setDebug("Outtake Velocity", drive.outtake.getVelocity(AngleUnit.RADIANS));
                    telemetry.setDebug("Ticks Outtake Velocity", drive.outtake.getVelocity(AngleUnit.DEGREES));
                    telemetry.setDebug("Ticks Outtake Position", drive.outtake.getCurrentPosition());
                }
                drive.followTrajectory(constantLaunchSpline2);
                break;
            }

            case TURN_POWER_SHOT:{
                drive.followTrajectory(turnLaunchSpline);
                while (drive.isBusy()){}
                drive.turn(Math.toRadians(5));
                while (drive.isBusy()){}
                launchRing(1, launchLineSpeed);
                drive.turn(Math.toRadians(5));
                while (drive.isBusy()){}
                launchRing(1, launchLineSpeed);
                break;
            }

            case POWERSHOT_MODE: {
                drive.wobbleGrab.setPower(1);
                drive.followTrajectory(powerLaunchSpline);
                while (drive.isBusy()){}
                drive.followTrajectory(secondPowerShot);
                while (drive.isBusy()){}
                drive.followTrajectory(thirdPowerShot);
                while (drive.isBusy()){}
                drive.outtake.setVelocity(0);
                drive.followTrajectory(fourSplinePower);
                while (drive.isBusy()){}
                drive.turn(Math.toRadians(-90));
                while (drive.isBusy()){}
                drive.liftWobble(-130, 0.45, AngleUnit.DEGREES, 1500);
                drive.wobbleGrab.setPower(-1);
                sleep(300);
                drive.wobbleGrab.setPower(0);
                sleep(200);
            break;
            }

            case BASE_MODE: {
        // Have the robot drive out
        drive.followTrajectory(launchSpline);

        while (drive.isBusy()) {
            telemetry.setDebug("Pose", drive.getPoseEstimate().toString());
        }

        // Start the outtake and wait for it to reach full speed
        drive.outtake.setVelocity(launchLineSpeed, AngleUnit.RADIANS);
        while (opModeIsActive() && Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) - launchLineSpeed) > Math.pow(10, -1)) {
            telemetry.setDebug("Outtake Velocity", drive.outtake.getVelocity(AngleUnit.RADIANS));
            telemetry.setDebug("Ticks Outtake Velocity", drive.outtake.getVelocity(AngleUnit.DEGREES));
            telemetry.setDebug("Ticks Outtake Position", drive.outtake.getCurrentPosition());
        }

        // Fire all the rings!
        int ringsFired = 0;
        while (opModeIsActive() && ringsFired <= 5) {
            telemetry.setDebug("ringsFired", ringsFired);
            drive.kicker.setPosition(.7);
            sleep(200);
            drive.kicker.setPosition(.3);
            sleep(200);
            while (Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) - launchLineSpeed) > Math.pow(10, -1)) {
                telemetry.setDebug("Outtake Velocity", drive.outtake.getVelocity(AngleUnit.RADIANS));
            }
            ringsFired++;
        }

        // Reset outtake
       // drive.outtake.setVelocity(0, AngleUnit.RADIANS);
       /* while (opModeIsActive() && Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS)) > Math.pow(10, -1)) {
            telemetry.setDebug("Outtake Velocity", drive.outtake.getVelocity(AngleUnit.RADIANS));
            telemetry.setDebug("Ticks Outtake Velocity", drive.outtake.getVelocity(AngleUnit.DEGREES));
            telemetry.setDebug("Ticks Outtake Position", drive.outtake.getCurrentPosition());
        }*/

        // Move wobble goal to target zone
        if (ringStack == 0) {
            drive.turn(Math.toRadians(-60));
            drive.followTrajectory(noSquare);
        } else if (ringStack == 1) {
            drive.followTrajectory(oneSpline);
        } else if (ringStack == 4) {
            //drive.followTrajectory(fourCollect);
            //drive.followTrajectory(fourCollect2);
            drive.followTrajectory(fourSpline);
        }
        drive.liftWobble(-125, 0.35, AngleUnit.DEGREES, 1500);
        drive.wobbleGrab.setPower(-1);
        sleep(400);
        drive.wobbleGrab.setPower(0);
        sleep(200);
        drive.liftWobble(-20,.35,AngleUnit.DEGREES,1500);
        if (ringStack == 0) {
            drive.followTrajectory(noRingDisengage);
        }else if(ringStack == 1){
            drive.followTrajectory(oneRingDisengage);
        }else if(ringStack == 4){
            drive.followTrajectory(fourBack);
        }
        /*if (ringStack == 4) {
            drive.followTrajectory(fourBack);
        } else*/
            if(ringStack == 0) {
                drive.followTrajectory(noRings);
            }else if (ringStack == 1){
                drive.followTrajectory(oneRingPickUp);
            }else if(ringStack == 4){
                drive.followTrajectory(fourRingPickUp);
            }
            sleep(300);
            drive.liftWobble(30, 0.45, AngleUnit.DEGREES, 1500);
                if (ringStack == 0) {
                    drive.followTrajectory(noRingsDrop);
                    waitForDrive();
                }else if(ringStack == 1){
                    drive.followTrajectory(oneRingSecondDrop);
                    waitForDrive();
                    drive.wobbleGrab.setPower(-1);
                    sleep(300);

                }else if(ringStack == 4){
                    drive.followTrajectory(fourRingSecondDrop);
                    drive.wobbleGrab.setPower(-1);
                    sleep(300);
                    drive.followTrajectory(fourBack2);
                }


        PoseStorage.currentPose = drive.getPoseEstimate();
        break;}
        }
    }

    private void waitForDrive() {
        // noinspection StatementWithEmptyBody
        while (this.drive.isBusy()) {}
    }

    private int getZoneFromOpenCV() {
        semaphore.waitForFrame();
        return stackDetector.getLastStackHeight();
    }
    private void launchRing(int ringsToFire, double speed){

            drive.outtake.setVelocity(speed, AngleUnit.RADIANS);
            while (opModeIsActive() && Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) - speed) > Math.pow(10, -1)) {
                telemetry.setDebug("Outtake Velocity", drive.outtake.getVelocity(AngleUnit.RADIANS));
                telemetry.setDebug("Ticks Outtake Velocity", drive.outtake.getVelocity(AngleUnit.DEGREES));
                telemetry.setDebug("Ticks Outtake Position", drive.outtake.getCurrentPosition());
            }
            int ringsFired = 0;
            while (opModeIsActive() && ringsFired < ringsToFire) {
                telemetry.setDebug("ringsFired", ringsFired);
                drive.kicker.setPosition(.7);
                sleep(200);
                drive.kicker.setPosition(.3);
                sleep(200);
                while (Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) - speed ) > Math.pow(10, -1)) {
                    telemetry.setDebug("Outtake Velocity", drive.outtake.getVelocity(AngleUnit.RADIANS));
                }
                ringsFired++;
            }
            drive.outtake.setVelocity(0);

    }

}
