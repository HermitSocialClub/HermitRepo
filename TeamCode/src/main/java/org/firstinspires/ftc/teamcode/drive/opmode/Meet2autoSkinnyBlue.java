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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
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

@Autonomous(name = "Meet2AutoSkinnyBlue")
public class Meet2autoSkinnyBlue extends LinearOpMode {

    private BaselineMecanumDrive drive;
    private PersistantTelemetry telemetry;

    Trajectory backUp;
    Trajectory toBlueHub;
    //    Trajectory toBlueBarrier;
    Trajectory cycleFromHub;
    Trajectory toBlueWarehouse;
    Trajectory blueWarehouseToHub;
//    Trajectory toBlueWarehouseBack;
//    Trajectory goBack;

    Pose2d blueStart = new Pose2d(6, 63.5, m(90));
    Vector2d blueHub = new Vector2d(-10, 44);
    Pose2d blueIntermediate = new Pose2d(6.25, 57, m(0));
    Pose2d blueBarrier = new Pose2d(12, 65.50, m(0));
    Vector2d blueWarehouse = new Vector2d(44, 65.50);
    Pose2d blueLeaveWarehouse = new Pose2d(40,65.50,0);
//    Pose2d blueCarousel = new Pose2d(-12,44,m(90));
//    Pose2d blueBarrier = new Pose2d(12,42,m(0));
//    Pose2d bluePit = new Pose2d(48,48,m(45));

    private VisionPipeline visionPipeline;
    private FirstFrameSemaphore semaphore;
    private BarcodeDetect detector;
    private Byte barcodeLevel;

    private ColorSensor color;

    private MotorConfigurationType carouselType;
    private double carouselSpeed = .3;
    private MotorConfigurationType liftType;
    private MotorConfigurationType intakeType;
    private Trajectory toBlueBarrierBack;
    LinearHelpers linear;
//    private ColorSensor color_in;


    ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new PersistantTelemetry(super.telemetry);
        drive = new BaselineMecanumDrive(hardwareMap, telemetry);


//        telemetry.setData("color_in Red",color_in.red());
//        telemetry.setData("color_in Blue",color_in.blue());
//        telemetry.setData("color_in Green",color_in.green());

        linear = new LinearHelpers(drive, telemetry);

        color = hardwareMap.get(ColorSensor.class, "color");

        drive.setPoseEstimate(blueStart);

        detector = new BarcodeDetect(true);
        this.semaphore = new FirstFrameSemaphore();
        this.visionPipeline = new VisionPipeline(hardwareMap, telemetry, detector, semaphore);
        CameraStreamSource cameraStream = visionPipeline.getCamera();
        FtcDashboard.getInstance().startCameraStream(cameraStream, 0);
        barcodeLevel = detector.getResult();



        toBlueHub = drive.trajectoryBuilder(blueStart)
                .addDisplacementMarker(() -> this.setLinearToBarcode())
                .strafeTo(blueHub)
                .build();

        toBlueWarehouse = drive.trajectoryBuilder(new Pose2d(blueHub, m(90)), m(50))
                .splineToSplineHeading(blueBarrier, m(50))
                .splineToConstantHeading(blueWarehouse, m(0))
                .build();

        cycleFromHub = drive.trajectoryBuilder(new Pose2d(blueHub, m(90)), m(50))
                .addDisplacementMarker(() -> linear.setLinears(0))
//                .addTemporalMarker(1.5, () -> drive.lift.setVelocity(0))
                .splineToSplineHeading(blueBarrier, m(50))
                .addDisplacementMarker(() -> {
                    drive.intake.setVelocity(1
                            * intakeType.getAchieveableMaxRPMFraction() *
                            intakeType.getMaxRPM() / 60 * Math.PI * 2, AngleUnit.RADIANS);
                    drive.stopFollowing();
//                        while (drive.colorSensor.red() < 50){
//                            drive.update();
//                        }
//                        drive.intake.setPower(0);

                })
                .splineToConstantHeading(blueWarehouse, m(0))
//                .addDisplacementMarker(() -> drive.stopFollowing())
                .build();


        blueWarehouseToHub = drive.trajectoryBuilder(blueLeaveWarehouse,m(160))
                .splineToConstantHeading(new Vector2d(blueBarrier.getX(), blueBarrier.getY()), m(200))
                .addDisplacementMarker(() -> {
                    drive.intake.setVelocity(-1
                            * intakeType.getAchieveableMaxRPMFraction() *
                            intakeType.getMaxRPM() / 60 * Math.PI * 2, AngleUnit.RADIANS);
                    linear.setLinears(3);
                })
                .splineToSplineHeading(new Pose2d(blueHub, m(90)), m(225))
                .addDisplacementMarker(() -> {
                    ElapsedTime time = new ElapsedTime();
                    drive.outtakeArm.setPosition(0.35);
                    time.reset();
                    while (time.milliseconds() < 900) {
                        drive.update();
                    }
                    drive.outtakeArm.setPosition(1);
                })

                .build();


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
        while (!isStarted()){
            barcodeLevel = detector.getResult();
            RobotLog.d(barcodeLevel.toString());
        }

        waitForStart();

        barcodeLevel = detector.getResult();

//        while(opModeIsActive()){
//            drive.lift.setTargetPosition((int) 0.75 * linear.TICKS_PER_REV + linear.startingPosition);
//            drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            drive.lift.setPower(0.95);
//        }


//
        drive.followTrajectoryAsync(toBlueHub);
        while (drive.isBusy() && !Thread.currentThread().isInterrupted()) {
            drive.update();
            linear.LinearUpdate();
        }
        drive.outtakeArm.setPosition(0.35);
        sleep(700);
        drive.outtakeArm.setPosition(1);
        for (int i = 0; i < 4; i++) {
            telemetry.setData("Cycle Number: ", i + 1);
            drive.followTrajectoryAsync(cycleFromHub);
            time.reset();
            while (!Thread.currentThread().isInterrupted() && drive.isBusy()) {
                drive.update();
                linear.LinearUpdate();
            }
//            drive.setWeightedDrivePower(new Pose2d(0.6,0,0));
            boolean forward = true;
            while (opModeIsActive() && color.red() < 80){
                Pose2d estimate = drive.getPoseEstimate();
                Pose2d d = estimate.minus(blueLeaveWarehouse);
                telemetry.setData("Distance from Target Pose",
                        Math.hypot(d.getX(),d.getY()));
                telemetry.setData("Robot Pose Velocity",drive.getPoseVelocity().toString());
                if (estimate.getX() >= 52) {
                    forward = false;
                }
                if (estimate.getX() <= 40) {
                    forward = true;
                }
                if(forward) {
                    drive.setWeightedDrivePower(new Pose2d(1.2,0,0));
                }
                else {
                    drive.setWeightedDrivePower(new Pose2d(-0.8,0,0));
                }

            }
            drive.intake.setVelocity(-1
                    * intakeType.getAchieveableMaxRPMFraction() *
                    intakeType.getMaxRPM() / 60 * Math.PI * 2, AngleUnit.RADIANS);
            drive.setWeightedDrivePower(new Pose2d());

            drive.followTrajectoryAsync(blueWarehouseToHub);
            time.reset();
            while (!Thread.currentThread().isInterrupted() && drive.isBusy()) {
                drive.update();
                linear.LinearUpdate();
            }

//            sleep(900);
        }

//        drive.followTrajectory(toBlueWarehouse);
//        while(opModeIsActive()){
//
//        }

    }
    public void setLinearToBarcode() {
        linear.setLinears(barcodeLevel == 4 ? 3 : barcodeLevel);
    }
}
