package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.hermitsocialclub.util.MoveUtils.m;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.hydra.vision.FirstFrameSemaphore;
import org.hermitsocialclub.hydra.vision.VisionPipeline;
import org.hermitsocialclub.telecat.PersistantTelemetry;
import org.hermitsocialclub.tomato.BarcodeDetect;
import org.hermitsocialclub.util.LinearHelpers;

@Autonomous(name = "Meet2autoducksred")
public class Meet2DucksRed extends LinearOpMode {

    private BaselineMecanumDrive drive;
    private PersistantTelemetry telemetry;

    private VisionPipeline visionPipeline;
    private FirstFrameSemaphore semaphore;
    private BarcodeDetect detector;
    private Byte barcodeLevel;

    LinearHelpers linear;

    private MotorConfigurationType intakeType;

    private double carouselSpeed = .85;
    private MotorConfigurationType carouselType;

    Pose2d redStart = new Pose2d(-42,-64,m(-90));
    Vector2d redHub = new Vector2d(-10, -44);
    Vector2d redCarousel = new Vector2d(-58.5,-59);

    Trajectory redHubTraj;
    Trajectory redCarouselTraj;
    Trajectory toRedCarousel;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new PersistantTelemetry(super.telemetry);
        drive = new BaselineMecanumDrive(hardwareMap, telemetry);

        carouselType = drive.duck_wheel.getMotorType();
        drive.duck_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linear = new LinearHelpers(drive, telemetry);

        intakeType = drive.intake.getMotorType();

        detector = new BarcodeDetect(true);
        this.semaphore = new FirstFrameSemaphore();
        this.visionPipeline = new VisionPipeline(hardwareMap, telemetry, detector, semaphore);
        CameraStreamSource cameraStream = visionPipeline.getCamera();
        FtcDashboard.getInstance().startCameraStream(cameraStream, 0);
        barcodeLevel = detector.getResult();

        drive.setPoseEstimate(redStart);

        redHubTraj = drive.trajectoryBuilder(redStart,m(45))
                .addDisplacementMarker(() -> setLinearToBarcode())
                .strafeTo(redHub)
//                .addDisplacementMarker(() -> drive.outtakeArm.setPosition(0))
                .build();

        toRedCarousel = drive.trajectoryBuilder(new Pose2d(redHub.getX(), redHub.getY(), m(-90)))
                .lineToSplineHeading(new Pose2d(redCarousel.getX(), redCarousel.getY(), m(-30)))
                .build();

        redCarouselTraj = drive.trajectoryBuilder(new Pose2d(redHub,m(90)),m(163))
                .splineToConstantHeading(redCarousel,m(165))
                .addDisplacementMarker(() -> {
                    drive.outtakeArm.setPosition(0.55);
                    linear.setLinears(0);
                    drive.intake.setVelocity(1
                        * intakeType.getAchieveableMaxRPMFraction() *
                        intakeType.getMaxRPM() / 60 * Math.PI * 2, AngleUnit.RADIANS);
                    drive.duck_wheel.setVelocity(carouselSpeed *
                            carouselType.getAchieveableMaxRPMFraction()
                            * carouselType.getMaxRPM()/60 *
                            Math.PI * 2, AngleUnit.RADIANS);
                })
                .build();

        while (!isStarted()){
            barcodeLevel = detector.getResult();
        }

        waitForStart();

        drive.followTrajectoryAsync(redHubTraj);
        while (drive.isBusy() && !Thread.currentThread().isInterrupted()){
            drive.update();
            linear.LinearUpdate();
        }
        drive.outtakeArm.setPosition(0.05);
        sleep(700);
        drive.outtakeArm.setPosition(1);
        drive.followTrajectoryAsync(toRedCarousel);
        while (drive.isBusy() && !Thread.currentThread().isInterrupted()){
            drive.update();
            linear.LinearUpdate();
        }
//        sleep(2200);
//        drive.duck_wheel.setVelocity(0);
//        sleep(500);


    }

    public void setLinearToBarcode() {
        linear.setLinears(barcodeLevel == 3 ? 3 : barcodeLevel);
    }
}
