package org.hermitsocialclub.opmodes.pandemicpanic;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.opmodes.freightfrenzy.PoseStorage;
import org.hermitsocialclub.hydra.opmodes.SubmatSetupOp;
import org.hermitsocialclub.hydra.vision.StaccDetecc;
import org.hermitsocialclub.hydra.vision.VisionPipeline;
import org.hermitsocialclub.hydra.vision.VisionSemaphore;
import org.hermitsocialclub.hydra.vision.util.VisionUtils;
import org.hermitsocialclub.telecat.PersistantTelemetry;

import static org.hermitsocialclub.drive.config.Meet0BotConstants.*;

@Disabled
@Autonomous(name = "Scrimmage5Auto")
public class Scrimmage5Auto extends LinearOpMode {

    private static final double POWER_SHOT_PERCENT = 0.575;
    private static final double SPEED_PERCENT = 0.755;
    private static final double COLLECT_LAUNCH_PERCENT = 0.645;
    private static final double COLLECT_LATE_LAUNCH_PERCENT = .665;
    private static final double BACK_PERCENT = .72;
    private static final double INTAKE_PERCENT = .55;
    private PersistantTelemetry telemetry;
    private MotorConfigurationType goBildaOutTake;
    private BaselineMecanumDrive drive;
    private double launchLineSpeed;
    private double collectLaunchSpeed;
    private double collectLateLaunchSpeed;
    private double powerShotSpeed;
    private double intakeSpeed;
    private double backSpeed;
    private int ringStack = 0;
    private StaccDetecc stackDetector;
    private VisionSemaphore semaphore;
    private VisionPipeline visionPipeline;
    private ElapsedTime time;

    private enum Ambition {
        BASE_MODE, POWERSHOT_MODE, TURN_POWER_SHOT, CONSTANT_POWER_SHOT, BROKEN, FRONT_HIGH_SHOT
    }

    private final Ambition ambition = Ambition.FRONT_HIGH_SHOT;

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
        this.intakeSpeed = ((INTAKE_PERCENT * 2 * Math.PI * goBildaOutTake.getMaxRPM() *
                goBildaOutTake.getAchieveableMaxRPMFraction()) / 60);
        this.backSpeed = ((BACK_PERCENT * 2 * Math.PI * goBildaOutTake.getMaxRPM() *
                goBildaOutTake.getAchieveableMaxRPMFraction()) / 60);
        this.drive = new BaselineMecanumDrive(hardwareMap, telemetry);
        this.drive.setPoseEstimate(new Pose2d(-60, -15.5, 0));

        this.stackDetector = new StaccDetecc();
        this.stackDetector.getConfig().setSubmat(VisionUtils.loadRectFromFile(SubmatSetupOp.SUBMAT_CONFIG));
        this.semaphore = new VisionSemaphore();
        this.visionPipeline = new VisionPipeline(hardwareMap, telemetry, stackDetector, semaphore);
        this.time = new ElapsedTime();
        this.time.reset();

        drive.wobbleGrab.setPosition(0);
        drive.wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Find which Zone the ring stack is in
        CameraStreamSource cameraStream = visionPipeline.getCamera();
        FtcDashboard.getInstance().startCameraStream(cameraStream, 0);
        while (time.seconds() < 4 && !isStarted() && !isStopRequested()) {
            ringStack = getZoneFromOpenCV();
            telemetry.setDebug("detected", ringStack);
        }
        telemetry.setDebug("done", "viewing");
        telemetry.setDebug("final position", ringStack);
        drive.wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Build trajectories

        Trajectory frontInitialShot;
        Trajectory frontCollect;
        Trajectory frontSecondShot;
        Trajectory frontWobbleFourZone1;
        Trajectory frontWobbleOneZone1;
        Trajectory frontWobbleFourZone1Disengage;
        Trajectory frontWobbleCollect;
        Trajectory frontWobbleFourZone2;
        Trajectory frontBack;
        //Front High Goal
        {
            frontInitialShot = drive
                    .trajectoryBuilder(drive.getPoseEstimate(), Math.toRadians(30))
                    .addDisplacementMarker(() -> drive.wobbleGrab.setPosition(0))
                    .splineToConstantHeading(new Vector2d(0, -41), Math.toRadians(-120))
                    .build();
            frontCollect = drive.trajectoryBuilder(frontInitialShot.end(), Math.toRadians(-55))
                    .addDisplacementMarker(() -> drive.intake.setVelocity(-intakeSpeed, AngleUnit.RADIANS))
                    .splineToConstantHeading(new Vector2d(-18, -48), 0,
                            new MecanumConstraints(
                                    new DriveConstraints(30,
                                            MAX_ACCEL,
                                            0,
                                            MAX_ANG_VELO,
                                            MAX_ANG_ACCEL,
                                            0),
                                    TRACK_WIDTH
                            ))
                    .addDisplacementMarker(() -> drive.intake.setVelocity(-intakeSpeed, AngleUnit.RADIANS))
                    .splineToConstantHeading(new Vector2d(6, -47), 0,
                            new MecanumConstraints(
                                    new DriveConstraints(30,
                                            MAX_ACCEL,
                                            0,
                                            MAX_ANG_VELO,
                                            MAX_ANG_ACCEL,
                                            0),
                                    TRACK_WIDTH
                            ))
                    .build();
            frontSecondShot = drive.trajectoryBuilder(frontCollect.end())
                    .splineToConstantHeading(new Vector2d(-3, -41), 0)
                    .build();
            frontWobbleFourZone1 = drive.trajectoryBuilder(frontInitialShot.end(), Math.toRadians(0))
                    .splineToLinearHeading(

                            new Pose2d(50, -63, Math.toRadians(180)), Math.toRadians(-75)

                    )
                    .addDisplacementMarker(() -> drive.wobbleGrab.setPosition(1))
                    .build();
            frontWobbleFourZone1Disengage = drive.trajectoryBuilder(frontWobbleFourZone1.end())
                    .back(2)
                    .build();
            frontWobbleCollect = drive.trajectoryBuilder(frontWobbleFourZone1Disengage.end(), Math.toRadians(180))
                    .splineToLinearHeading(new Pose2d(-37, -43, 0), Math.toRadians(160))
                    .addDisplacementMarker(() -> drive.wobbleGrab.setPosition(0))
                    .build();
            frontWobbleFourZone2 = drive.trajectoryBuilder(frontWobbleCollect.end())
                    .splineToLinearHeading(
                            new Pose2d(47, -63, Math.toRadians(180)),
                            0)
                    .addDisplacementMarker(() -> drive.wobbleGrab.setPosition(1))
                    .build();
            frontBack = drive.trajectoryBuilder(frontWobbleFourZone2.end())
                    .back(.5)
                    .splineToConstantHeading(new Vector2d(10, -53), Math.toRadians(180))
                    .build();
        }

        Trajectory constantLaunchSpline;
        Trajectory constantLaunchSpline2;
        Trajectory constantCollectSetup;
        Trajectory constantCollect;
        Trajectory constantFirstFourWobbleDrop;
        Trajectory constantWobbleCollect;
        Trajectory constantSecondFourWobbleDrop;
        //Constant Power Shot
        {
            constantLaunchSpline = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineToConstantHeading(new Vector2d(-14, 12.50), 0)
                    .addDisplacementMarker(() -> {
                        drive.hopperLift.setPosition(.385);
                        drive.outtake.setVelocity(powerShotSpeed);
                    })
                    .splineToConstantHeading(new Vector2d(-3, -24.50), 0)
                    .addSpatialMarker(new Vector2d(0, -2.50), () -> launchRing(1, powerShotSpeed))
                    .addSpatialMarker(new Vector2d(0, -14.50), () -> launchRing(1, powerShotSpeed))
                    .addSpatialMarker(new Vector2d(0, -20.50), () -> launchRing(1, powerShotSpeed))
                    .splineToConstantHeading(new Vector2d(-3, -16.5), Math.toRadians(90))
                    //.splineToConstantHeading(new Vector2d(-36,-36.5),Math.toRadians(-50))
                    //.addDisplacementMarker(()->drive.intake.setVelocity(intakeSpeed,AngleUnit.RADIANS))
                    //.splineToConstantHeading(new Vector2d(-10,-36.5),0)
                    .build();
            constantFirstFourWobbleDrop = drive.trajectoryBuilder(constantLaunchSpline.end())
                    .splineToLinearHeading(new Pose2d(55, -58, Math.toRadians(180)), Math.toRadians(-55))
                    .addDisplacementMarker(() -> drive.wobbleGrab.setPosition(0))
                    .build();
            constantWobbleCollect = drive.trajectoryBuilder(constantFirstFourWobbleDrop.end(), Math.toRadians(180))
                    .splineToLinearHeading(new Pose2d(-36, -53, 0), Math.toRadians(180))
                    .addDisplacementMarker(() -> drive.wobbleGrab.setPosition(1))
                    .build();
            constantSecondFourWobbleDrop = drive.trajectoryBuilder(constantWobbleCollect.end())
                    .splineToLinearHeading(new Pose2d(55, -53, Math.toRadians(180)), 0)
                    .addDisplacementMarker(() -> drive.wobbleGrab.setPosition(0))
                    .build();
        }
        Trajectory turnLaunchSpline;
        //Turn PowerShot
        {
            turnLaunchSpline = drive
                    .trajectoryBuilder(drive.getPoseEstimate())
                    .addDisplacementMarker(() -> drive.wobbleGrab.setPosition(1))
                    .splineToLinearHeading(new Pose2d(-3, -42, Math.toRadians(12.5)), 60)
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
        }
        Trajectory powerLaunchSpline;
        Trajectory secondPowerShot;
        Trajectory thirdPowerShot;
        Trajectory fourSplinePower;
        //Standard Powershot
        {
            powerLaunchSpline = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(-34, -2, 0), 0)
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
            secondPowerShot = drive.trajectoryBuilder(powerLaunchSpline.end())
                    .lineToConstantHeading(new Vector2d(-34, -14.50))
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
            thirdPowerShot = drive.trajectoryBuilder(secondPowerShot.end())
                    .lineToConstantHeading(new Vector2d(-34, -20.50))
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
            fourSplinePower = drive.trajectoryBuilder(thirdPowerShot.end())
                    .splineToLinearHeading(new Pose2d(50, -36, 270), 0)
                    .build();
        }
        Trajectory launchSpline;
        Trajectory oneSpline;
        Trajectory fourSpline;
        Trajectory noSquare;
        Trajectory noRingDisengage;
        Trajectory noRings;
        Trajectory noRingsDrop;
        Trajectory oneRingDisengage;
        Trajectory oneRingPickUp;
        Trajectory oneRingSecondDrop;
        Trajectory fourRingSecondDrop;
        Trajectory fourBack;
        Trajectory fourBack2;
        Trajectory fourRingPickUp;
        //Standard High Goal
        {
            launchSpline = drive
                    .trajectoryBuilder(drive.getPoseEstimate())
                    .addDisplacementMarker(() -> drive.wobbleGrab.setPosition(1))
                    .splineToConstantHeading(new Vector2d(-3, -41), Math.toRadians(-80))
                    .build();
            oneSpline = drive
                    .trajectoryBuilder(launchSpline.end())
                    .forward(12)
                    .build();
            fourSpline = drive
                    .trajectoryBuilder(launchSpline.end())
                    .splineToConstantHeading(new Vector2d(38, -58), 0)
                    .build();

            noSquare = drive
                    .trajectoryBuilder(launchSpline.end().plus(new Pose2d(0, 0, Math.toRadians(-60))))
                    .forward(5.5)
                    .build();
            noRingDisengage = drive
                    .trajectoryBuilder(noSquare.end())
                    .back(9)
                    .build();

            noRings = drive
                    .trajectoryBuilder(noRingDisengage.end(), Math.toRadians(120))
                    .splineToLinearHeading(new Pose2d(-30.50, -42.50, Math.toRadians(180)), Math.toRadians(180))
                    .addDisplacementMarker(() -> drive.wobbleGrab.setPosition(1))
                    .build();
            noRingsDrop = drive
                    .trajectoryBuilder(noRings.end(), Math.toRadians(180))
                    .splineToLinearHeading(new Pose2d(8.50, -56.50, Math.toRadians(-30)), 0)
                    .build();
            oneRingDisengage = drive.trajectoryBuilder(oneSpline.end())
                    .back(9)
                    .build();
            oneRingPickUp = drive.trajectoryBuilder(oneRingDisengage.end())
                    .splineToLinearHeading(new Pose2d(-36.50, -46.50, Math.toRadians(180)), Math.toRadians(180))
                    .build();
            oneRingSecondDrop = drive.trajectoryBuilder(noRings.end())
                    .splineToLinearHeading(new Pose2d(12, -36.50, 0), 0)
                    .build();
            fourRingSecondDrop = drive.trajectoryBuilder(noRings.end())
                    .splineToLinearHeading(new Pose2d(36.50, -58, 0), 0)
                    .addDisplacementMarker(() -> drive.wobbleGrab.setPosition(-1))
                    .build();
            fourBack = drive
                    .trajectoryBuilder(fourSpline.end())
                    .back(9)
                    .build();
            fourBack2 = drive
                    .trajectoryBuilder(fourRingSecondDrop.end())
                    .back(20)
                    .build();
            fourRingPickUp = drive.trajectoryBuilder(fourBack.end(), Math.toRadians(180))
                    .splineToLinearHeading(new Pose2d(-36.50, -46.50, Math.toRadians(180)), Math.toRadians(180))
                    .build();
        }
        Trajectory failure1;
        Trajectory failureZero0;
        Trajectory failureZero;
        Trajectory failureOne;
        Trajectory failureFour;
        Trajectory failureFourBack;
        //failure
        {
            failure1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .addDisplacementMarker(() -> drive.wobbleGrab.setPosition(1))
                    .forward(62.5)
                    .build();

            failureZero0 = drive.trajectoryBuilder(failure1.end())
                    .forward(5)
                    .build();
            failureZero = drive.trajectoryBuilder(failureZero0.end().plus(new Pose2d(0, 0, Math.toRadians(120))))
                    .forward(4)
                    .build();
            failureOne = drive.trajectoryBuilder(failure1.end())
                    .forward(18)
                    .build();
            failureFour = drive.trajectoryBuilder(failure1.end())
                    .forward(48)
                    .build();
            failureFourBack = drive.trajectoryBuilder(failureFour.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                    .forward(36)
                    .build();
        }
        telemetry.setDebug("Trajectories", "Done");

        waitForStart();

        switch (ambition) {
            case FRONT_HIGH_SHOT: {
                drive.followTrajectory(frontInitialShot);
                drive.hopperLift.setPosition(.385);
                launchRing(3, launchLineSpeed);
                //drive.followTrajectory(frontCollect);
                //drive.followTrajectory(frontSecondShot);
                //
                //drive.intake.setVelocity(0);
                //sleep(300);
                drive.followTrajectory(frontWobbleFourZone1);
                drive.followTrajectory(frontWobbleFourZone1Disengage);
                drive.followTrajectory(frontWobbleCollect);
                drive.followTrajectory(frontWobbleFourZone2);
                drive.followTrajectory(frontBack);
                PoseStorage.currentPose = drive.getPoseEstimate();
                break;
            }
            case BROKEN: {
                drive.followTrajectory(failure1);
                launchRing(3, launchLineSpeed);
                if (ringStack == 1) {
                    drive.followTrajectory(failureOne);
                    drive.turn(Math.toRadians(180));
                    waitForDrive();
                    drive.wobbleGrab.setPosition(-1);
                    sleep(600);
                } else if (ringStack == 0) {
                    drive.followTrajectory(failureZero0);
                    drive.turn(Math.toRadians(120));
                    waitForDrive();
                    drive.wobbleGrab.setPosition(-1);
                    sleep(600);
                    drive.followTrajectory(failureZero);
                } else if (ringStack == 4) {
                    drive.followTrajectory(failureFour);
                    drive.turn(Math.toRadians(120));
                    waitForDrive();
                    drive.liftWobble(-125, .35, AngleUnit.DEGREES, 1500);
                    drive.wobbleGrab.setPosition(-1);
                    sleep(600);
                    drive.turn(Math.toRadians(60));
                    drive.followTrajectory(failureFourBack);
                }
                break;
            }
            case CONSTANT_POWER_SHOT: {
                drive.followTrajectory(constantLaunchSpline);
                //drive.outtake.setVelocity(powerShotSpeed);
                /*while (opModeIsActive() && Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) - powerShotSpeed) > Math.pow(10, -1)) {
                    telemetry.setDebug("Outtake Velocity", drive.outtake.getVelocity(AngleUnit.RADIANS));
                    telemetry.setDebug("Ticks Outtake Velocity", drive.outtake.getVelocity(AngleUnit.DEGREES));
                    telemetry.setDebug("Ticks Outtake Position", drive.outtake.getCurrentPosition());
                }*/
                //drive.followTrajectory(constantLaunchSpline2);
                break;
            }
            case TURN_POWER_SHOT: {
                drive.followTrajectory(turnLaunchSpline);
                while (drive.isBusy()) {
                }
                drive.turn(Math.toRadians(5));
                while (drive.isBusy()) {
                }
                launchRing(1, launchLineSpeed);
                drive.turn(Math.toRadians(5));
                while (drive.isBusy()) {
                }
                launchRing(1, launchLineSpeed);
                break;
            }
            case POWERSHOT_MODE: {
                drive.wobbleGrab.setPosition(1);
                drive.followTrajectory(powerLaunchSpline);
                while (drive.isBusy()) {
                }
                drive.followTrajectory(secondPowerShot);
                while (drive.isBusy()) {
                }
                drive.followTrajectory(thirdPowerShot);
                while (drive.isBusy()) {
                }
                drive.outtake.setVelocity(0);
                drive.followTrajectory(fourSplinePower);
                while (drive.isBusy()) {
                }
                drive.turn(Math.toRadians(-90));
                while (drive.isBusy()) {
                }
                drive.liftWobble(-130, 0.45, AngleUnit.DEGREES, 1500);
                drive.wobbleGrab.setPosition(-1);
                sleep(300);
                drive.wobbleGrab.setPosition(0);
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
                drive.wobbleGrab.setPosition(-1);
                sleep(400);
                drive.wobbleGrab.setPosition(0);
                sleep(200);
                drive.liftWobble(-20, .35, AngleUnit.DEGREES, 1500);
                if (ringStack == 0) {
                    drive.followTrajectory(noRingDisengage);
                } else if (ringStack == 1) {
                    drive.followTrajectory(oneRingDisengage);
                } else if (ringStack == 4) {
                    drive.followTrajectory(fourBack);
                }
        /*if (ringStack == 4) {
            drive.followTrajectory(fourBack);
        } else*/
                if (ringStack == 0) {
                    drive.followTrajectory(noRings);
                } else if (ringStack == 1) {
                    drive.followTrajectory(oneRingPickUp);
                } else if (ringStack == 4) {
                    drive.followTrajectory(fourRingPickUp);
                }
                sleep(300);
                drive.liftWobble(30, 0.45, AngleUnit.DEGREES, 1500);
                if (ringStack == 0) {
                    drive.followTrajectory(noRingsDrop);
                    waitForDrive();
                } else if (ringStack == 1) {
                    drive.followTrajectory(oneRingSecondDrop);
                    waitForDrive();
                    drive.wobbleGrab.setPosition(-1);
                    sleep(300);

                } else if (ringStack == 4) {
                    drive.followTrajectory(fourRingSecondDrop);
                    drive.wobbleGrab.setPosition(-1);
                    sleep(300);
                    drive.followTrajectory(fourBack2);
                }


                break;
            }
        }
        PoseStorage.currentPose = drive.getPoseEstimate();

    }

    private void waitForDrive() {
        // noinspection StatementWithEmptyBody
        while (this.drive.isBusy()) {
        }
    }

    private int getZoneFromOpenCV() {
        semaphore.waitForFrame();
        return stackDetector.getLastStackHeight();
    }

    private void launchRing(int ringsToFire, double speed) {

        drive.outtake.setVelocity(speed, AngleUnit.RADIANS);
        while (opModeIsActive() && Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) - speed) > Math.pow(10, -1)) {
            telemetry.setDebug("Outtake Velocity", drive.outtake.getVelocity(AngleUnit.RADIANS));
            telemetry.setDebug("Ticks Outtake Velocity", drive.outtake.getVelocity(AngleUnit.DEGREES));
            telemetry.setDebug("Ticks Outtake Position", drive.outtake.getCurrentPosition());
        }
        int ringsFired = 0;
        while (opModeIsActive() && ringsFired < ringsToFire) {
            telemetry.setDebug("ringsFired", ringsFired);
            drive.kicker.setPosition(1);
            sleep(500);
            drive.kicker.setPosition(0);
            sleep(500);
            while (Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) - speed) > Math.pow(10, -2) * 5) {
                telemetry.setDebug("Outtake Velocity", drive.outtake.getVelocity(AngleUnit.RADIANS));
            }
            ringsFired++;
        }
        drive.outtake.setVelocity(0);

    }

}
