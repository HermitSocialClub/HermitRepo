package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.slamra;
import static org.hermitsocialclub.util.MoveUtils.m;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.hydra.vision.FirstFrameSemaphore;
import org.hermitsocialclub.hydra.vision.VisionPipeline;
import org.hermitsocialclub.telecat.PersistantTelemetry;
import org.hermitsocialclub.tomato.BarcodeDetect;
import org.hermitsocialclub.util.LinearHelpers;

@Autonomous(name = "Meet2autoducksblue")
public class Meet2_ducksBlue extends LinearOpMode {

    private BaselineMecanumDrive drive;
    private PersistantTelemetry telemetry;

    private ElapsedTime gameTime = new ElapsedTime();

    private VisionPipeline visionPipeline;
    private FirstFrameSemaphore semaphore;
    private BarcodeDetect detector;
    private Byte barcodeLevel;

    LinearHelpers linear;

    private MotorConfigurationType intakeType;

    private double carouselSpeed = -.70;
    private MotorConfigurationType carouselType;

    Pose2d blueStart = new Pose2d(-42,64,m(90));
    Vector2d blueHub = new Vector2d(-10, 43);
    Vector2d blueCarousel = new Vector2d(-59,60);
    Pose2d storageUnit = new Pose2d(-60,36, m(0));
    Pose2d LA_CANADA_GAMBIT = new Pose2d(36,65.5, m(0));

    Trajectory blueHubTraj;
    Trajectory blueCarouselTraj;
    Trajectory toblueCarousel;
    Trajectory toStorageUnit;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new PersistantTelemetry(super.telemetry);
        drive = new BaselineMecanumDrive(hardwareMap, telemetry);

        carouselType = drive.duck_wheel.getMotorType();
        drive.duck_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linear = new LinearHelpers(drive, telemetry,gameTime);
        linear.setMode(LinearHelpers.MODE.AUTON);

        intakeType = drive.intake.getMotorType();


        while(!gamepad1.x){
            telemetry.setData("Confidence", slamra.getLastReceivedCameraUpdate().confidence.toString());
            telemetry.setData("Press X to: ", "exit calibration when confidence is high");
        }
        telemetry.setData("left: ", "calibration sequence");

        detector = new BarcodeDetect(true);
        this.semaphore = new FirstFrameSemaphore();
        this.visionPipeline = new VisionPipeline(hardwareMap, telemetry, detector, semaphore);
        CameraStreamSource cameraStream = visionPipeline.getCamera();
        FtcDashboard.getInstance().startCameraStream(cameraStream, 0);
        barcodeLevel = detector.getResult();

        drive.setPoseEstimate(blueStart);

        blueHubTraj = drive.trajectoryBuilder(blueStart,m(45))
                .addDisplacementMarker(() -> setLinearToBarcode())
                .strafeTo(blueHub)
//                .addDisplacementMarker(() -> drive.outtakeArm.setPosition(0))
                .build();

        toblueCarousel = drive.trajectoryBuilder(new Pose2d(blueHub.getX(), blueHub.getY(), m(90)))
                .lineToSplineHeading(new Pose2d(blueCarousel.getX(), blueCarousel.getY(), m(135)))
                .build();

        blueCarouselTraj = drive.trajectoryBuilder(new Pose2d(blueHub,m(90)),m(163))
                .splineToConstantHeading(blueCarousel,m(165))
                .addDisplacementMarker(() -> {
                    drive.outtakeArm.setPosition(0.55);
                    linear.setLevel(LinearHelpers.LEVEL.ZERO);
                    drive.intake.setVelocity(1
                            * intakeType.getAchieveableMaxRPMFraction() *
                            intakeType.getMaxRPM() / 60 * Math.PI * 2, AngleUnit.RADIANS);
                    drive.duck_wheel.setVelocity(carouselSpeed *
                            carouselType.getAchieveableMaxRPMFraction()
                            * carouselType.getMaxRPM()/60 *
                            Math.PI * 2, AngleUnit.RADIANS);
                })
                .build();

        toStorageUnit = drive.trajectoryBuilder(new Pose2d(blueCarousel.getX(), blueCarousel.getY(), m(135)))
                .lineToLinearHeading(storageUnit)
                .build();

        while (!isStarted()){
            barcodeLevel = detector.getResult();
        }

        waitForStart();
        gameTime.reset();
        sleep(4000);

        drive.followTrajectoryAsync(blueHubTraj);
        while (drive.isBusy() && !Thread.currentThread().isInterrupted()){
            drive.update();
            linear.LinearUpdateNew();
        }
        drive.outtakeArm.setPosition(0.40);
        sleep(700);
        drive.outtakeArm.setPosition(1);
        drive.followTrajectoryAsync(toblueCarousel);
        while (drive.isBusy() && !Thread.currentThread().isInterrupted()){
            drive.update();
            linear.LinearUpdateNew();
        }
        drive.setWeightedDrivePower(new Pose2d(0.0125, 0.0, m(0)));
        drive.duck_wheel.setVelocity(carouselSpeed *
                carouselType.getAchieveableMaxRPMFraction()
                * carouselType.getMaxRPM()/60 *
                Math.PI * 2, AngleUnit.RADIANS);
//        drive.intake.setVelocity();
        sleep(500);
        drive.intake.setVelocity(0.65
                * intakeType.getAchieveableMaxRPMFraction() *
                intakeType.getMaxRPM() / 60 * Math.PI * 2, AngleUnit.RADIANS);
        sleep(2200);
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
//        sleep(1100);
        drive.duck_wheel.setPower(0);
//        while (gameTime)
//        sleep((((long)(2600 - ))));
        drive.followTrajectory(toStorageUnit);


    }

    public void setLinearToBarcode() {
        linear.setLevel(barcodeLevel == 4 ? LinearHelpers.LEVEL.THREE : LinearHelpers.LEVEL.values()[barcodeLevel]);
    }
}
