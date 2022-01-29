package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.hermitsocialclub.drive.config.DriveConstants.MAX_ACCEL;
import static org.hermitsocialclub.drive.config.DriveConstants.MAX_ANG_ACCEL;
import static org.hermitsocialclub.drive.config.DriveConstants.MAX_ANG_VELO;
import static org.hermitsocialclub.util.MoveUtils.m;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.hydra.opmodes.SubmatSetupOp;
import org.hermitsocialclub.hydra.vision.StaccDetecc;
import org.hermitsocialclub.hydra.vision.VisionPipeline;
import org.hermitsocialclub.hydra.vision.VisionSemaphore;
import org.hermitsocialclub.hydra.vision.util.VisionUtils;
import org.hermitsocialclub.telecat.PersistantTelemetry;
import org.hermitsocialclub.tomato.BarcodeDetect;
import org.hermitsocialclub.hydra.vision.FirstFrameSemaphore;
import org.hermitsocialclub.util.LinearHelpers;

import java.util.Vector;

@Autonomous(name = "Meet2AutoSkinny")
public class Meet2autoSkinny extends LinearOpMode {

    private BaselineMecanumDrive drive;
    private PersistantTelemetry telemetry;

    Trajectory backUp;
    Trajectory toBlueHub;
//    Trajectory toBlueBarrier;
    Trajectory cycleFromHub;
    Trajectory toBlueWarehouse;
//    Trajectory toBlueWarehouseBack;
//    Trajectory goBack;

    Pose2d blueStart =  new Pose2d(6,63.5,m(90));
    Vector2d blueHub = new Vector2d(-12, 46);
    Pose2d blueIntermediate = new Pose2d(6.25, 57, m(0));
    Pose2d blueBarrier = new Pose2d(12,65.50, m(0));
    Vector2d blueWarehouse = new Vector2d(48, 65.50);
//    Pose2d blueCarousel = new Pose2d(-12,44,m(90));
//    Pose2d blueBarrier = new Pose2d(12,42,m(0));
//    Pose2d bluePit = new Pose2d(48,48,m(45));

    private VisionPipeline visionPipeline;
    private FirstFrameSemaphore semaphore;
    private BarcodeDetect detector;
    private Byte barcodeLevel;

    private MotorConfigurationType carouselType;
    private double carouselSpeed = .3;
    private MotorConfigurationType liftType;
    private MotorConfigurationType intakeType;
    private Trajectory toBlueBarrierBack;
    LinearHelpers linear;

    ElapsedTime time = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new PersistantTelemetry(super.telemetry);
        drive = new BaselineMecanumDrive(hardwareMap,telemetry);

        linear = new LinearHelpers(drive,telemetry);


        drive.setPoseEstimate(blueStart);
        toBlueHub = drive.trajectoryBuilder(blueStart)
                .strafeTo(blueHub)
                .build();
        toBlueWarehouse = drive.trajectoryBuilder(new Pose2d(blueHub, m(90)), m(50))
                .splineToSplineHeading(blueBarrier, m(50))
                .splineToConstantHeading(blueWarehouse, m(0))
                .build();
        cycleFromHub = drive.trajectoryBuilder(new Pose2d(blueHub, m(90)), m(50))
                .addDisplacementMarker(() -> drive.lift.setVelocity(liftType
                        .getMaxRPM() / 60 * liftType.getAchieveableMaxRPMFraction() * Math.PI * 2 -.55 *
                        1, AngleUnit.RADIANS))
                .addTemporalMarker(1.5, () -> drive.lift.setVelocity(0))
                .splineToSplineHeading(blueBarrier, m(50))
                .addDisplacementMarker(() -> {drive.intake.setVelocity(1
                        * intakeType.getAchieveableMaxRPMFraction() *
                        intakeType.getMaxRPM() / 60 * Math.PI * 2, AngleUnit.RADIANS);
//                        while (drive.colorSensor.red() < 50){
//                            drive.update();
//                        }
//                        drive.intake.setPower(0);

                })
                .splineToConstantHeading(blueWarehouse, m(0))
                .splineToConstantHeading(new Vector2d(blueBarrier.getX(), blueBarrier.getY()), m(200))
                .addDisplacementMarker(()->drive.intake.setVelocity(0))
                .splineToSplineHeading(new Pose2d(blueHub, m(90)), m(-90))
                .addDisplacementMarker(() -> {
                    ElapsedTime time = new ElapsedTime();
                    drive.lift.setVelocity(liftType
                            .getMaxRPM() / 60 * liftType.getAchieveableMaxRPMFraction() * -.55 *
                            2 * Math.PI, AngleUnit.RADIANS);
                    while (time.milliseconds() < 2000){
                        drive.update();
                    }
                    drive.outtakeArm.setPosition(0);
                    time.reset();
                    while (time.milliseconds() < 900){
                        drive.update();
                    }
                    drive.lift.setVelocity(0);
                    drive.outtakeArm.setPosition(0.55);
                })

                .build();

        detector = new BarcodeDetect(true);
        this.semaphore = new FirstFrameSemaphore();
        this.visionPipeline = new VisionPipeline(hardwareMap, telemetry, detector, semaphore);

        barcodeLevel = detector.getResult();

        telemetry.setData("Barcode Level", barcodeLevel);
        CameraStreamSource cameraStream = visionPipeline.getCamera();
        FtcDashboard.getInstance().startCameraStream(cameraStream,0);

        carouselType = drive.duck_wheel.getMotorType();

        liftType = drive.lift.getMotorType();
        intakeType = drive.intake.getMotorType();

        liftType.getTicksPerRev();
        // find cur pos
        // add 300 fsm
        // run to position cur + 300
        //motor.setencodor
        // while motor is busy set power to .5

        drive.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.duck_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();


        drive.followTrajectory(toBlueHub);
        drive.lift.setVelocity(liftType
                .getMaxRPM() / 60 * liftType.getAchieveableMaxRPMFraction() * 2 * Math.PI *
                1, AngleUnit.RADIANS);
        sleep(1600);
        drive.outtakeArm.setPosition(0);
        drive.lift.setVelocity(liftType
                .getMaxRPM() / 60 * liftType.getAchieveableMaxRPMFraction() * .3,
                AngleUnit.RADIANS);
        sleep(900);
        drive.outtakeArm.setPosition(0.55);
        for (int i = 0; i < 3; i++){
            telemetry.setData("Cycle Number: ", i +1);
            drive.followTrajectory(cycleFromHub);
            sleep(900);
        }
        drive.followTrajectory(toBlueWarehouse);




//        time.reset();
//        while (!Thread.currentThread().isInterrupted() && drive.isBusy()){
//            drive.update();
////            linear.LinearUpdate();
//        }
    }
}
