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

@Autonomous(name = "Meet2AutoSkinnyRed")
public class Meet2autoSkinnyRed extends LinearOpMode {

    private BaselineMecanumDrive drive;
    private PersistantTelemetry telemetry;

    ElapsedTime gameTime = new ElapsedTime();

    Trajectory backUp;
    Trajectory toRedHub;
    //    Trajectory toBlueBarrier;
    Trajectory cycleFromHub;
//    Trajectory toRedWarehouse;
    Trajectory redWarehouseToHub;
//    Trajectory toBlueWarehouseBack;
//    Trajectory goBack;

    Pose2d redStart = new Pose2d(6, -63.5, m(-90));
    Vector2d redHub = new Vector2d(-14, -42);
//    Pose2d blueIntermediate = new Pose2d(6.25, 57, m(0));
    Pose2d redBarrier = new Pose2d(12, -65.50, m(0));
    Vector2d redWarehouse = new Vector2d(44, -65.50);
    Pose2d redLeaveWarehouse = new Pose2d(40,-66.50,0);
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

        linear = new LinearHelpers(drive, telemetry,gameTime);
        linear.setMode(LinearHelpers.MODE.AUTON);

        color = hardwareMap.get(ColorSensor.class, "color");

        drive.setPoseEstimate(redStart);

        detector = new BarcodeDetect(true);
        this.semaphore = new FirstFrameSemaphore();
        this.visionPipeline = new VisionPipeline(hardwareMap, telemetry, detector, semaphore);
        CameraStreamSource cameraStream = visionPipeline.getCamera();
        FtcDashboard.getInstance().startCameraStream(cameraStream, 0);
        barcodeLevel = detector.getResult();



        toRedHub = drive.trajectoryBuilder(redStart)
                .addDisplacementMarker(() -> this.setLinearToBarcode())
                .strafeTo(redHub)
                .build();
//
//        toRedWarehouse = drive.trajectoryBuilder(new Pose2d(redHub, m(90)), m(50))
//                .splineToSplineHeading(redBarrier, m(50))
//                .splineToConstantHeading(redWarehouse, m(0))
//                .build();

        cycleFromHub = drive.trajectoryBuilder(new Pose2d(redHub, m(-90)), m(-50))
                .addDisplacementMarker(() -> linear.setLevel(LinearHelpers.LEVEL.ZERO))
//                .addTemporalMarker(1.5, () -> drive.lift.setVelocity(0))
                .splineToSplineHeading(redBarrier, m(-50))
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
                .splineToConstantHeading(redWarehouse, m(0))
//                .addDisplacementMarker(() -> drive.stopFollowing())
                .build();


        redWarehouseToHub = drive.trajectoryBuilder(redLeaveWarehouse,m(-160))
                .splineToConstantHeading(new Vector2d(redBarrier.getX(), redBarrier.getY()), m(-200))
                .addDisplacementMarker(() -> {
                    drive.intake.setVelocity(-1
                            * intakeType.getAchieveableMaxRPMFraction() *
                            intakeType.getMaxRPM() / 60 * Math.PI * 2, AngleUnit.RADIANS);
                    linear.setLevel(LinearHelpers.LEVEL.THREE);
                })
                .splineToSplineHeading(new Pose2d(redHub, m(-90)), m(-225))
                .addDisplacementMarker(() -> {
                    drive.intake.setPower(0);
                })
                .addDisplacementMarker(() -> {
                    ElapsedTime time = new ElapsedTime();
                    drive.outtakeArm.setPosition(0.40);
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
        gameTime.reset();

        barcodeLevel = detector.getResult();

//        while(opModeIsActive()){
//            drive.lift.setTargetPosition((int) 0.75 * linear.TICKS_PER_REV + linear.startingPosition);
//            drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            drive.lift.setPower(0.95);
//        }


//
        drive.followTrajectoryAsync(toRedHub);
        while (drive.isBusy() && !Thread.currentThread().isInterrupted()) {
            drive.update();
            linear.LinearUpdateNew();
            BaselineMecanumDrive.poseEndingAuton = drive.getPoseEstimate();

        }
        drive.outtakeArm.setPosition(0.40);
        sleep(700);
        drive.outtakeArm.setPosition(1);
        for (int i = 0; i < 4; i++) {
            telemetry.setData("Cycle Number: ", i + 1);
            drive.followTrajectoryAsync(cycleFromHub);
            time.reset();
            while (!Thread.currentThread().isInterrupted() && drive.isBusy()) {
                drive.update();
                linear.LinearUpdateNew();

            }
//            drive.setWeightedDrivePower(new Pose2d(0.6,0,0));
            boolean forward = true;
            int modifier = 0;
            while (opModeIsActive() && color.red() < 80){
                Pose2d estimate = drive.getPoseEstimate();
                Pose2d d = estimate.minus(redLeaveWarehouse);
                telemetry.setData("Distance from Target Pose",
                        Math.hypot(d.getX(),d.getY()));
                telemetry.setData("Robot Pose Velocity",drive.getPoseVelocity().toString());
                if (estimate.getX() >= 50 + modifier) {
                    forward = false;
                    if (modifier != 4) modifier += 1;
                }
                if (estimate.getX() <= 42 + modifier) {
                    forward = true;
//                    if (modifier != 3) modifier += 1;

                }
                if(forward) {
                    drive.setWeightedDrivePower(new Pose2d(0.8,0,0));
                }
                else {
                    drive.setWeightedDrivePower(new Pose2d(-0.8,0,0));
                }
//                BaselineMecanumDrive.poseEndingAuton = drive.getPoseEstimate();

            }
            drive.intake.setVelocity(-0.85
                    * intakeType.getAchieveableMaxRPMFraction() *
                    intakeType.getMaxRPM() / 60 * Math.PI * 2, AngleUnit.RADIANS);
            drive.setWeightedDrivePower(new Pose2d());

            telemetry.setData("te e ", redWarehouseToHub.duration());
            if (30 - gameTime.seconds() < 4) {
//                BaselineMecanumDrive.poseEndingAuton = drive.getPoseEstimate();
                return;
            }
            drive.followTrajectoryAsync(redWarehouseToHub);
            time.reset();
            while (!Thread.currentThread().isInterrupted() && drive.isBusy()) {
                drive.update();
                linear.LinearUpdateNew();
        //        BaselineMecanumDrive.poseEndingAuton = drive.getPoseEstimate();
            }

//            sleep(900);
        }

//        drive.followTrajectory(toBlueWarehouse);
//        while(opModeIsActive()){
//
//        }

    }
    public void setLinearToBarcode() {
        linear.setLevel(barcodeLevel == 4 ? LinearHelpers.LEVEL.THREE : LinearHelpers.LEVEL.values()[barcodeLevel]);
    }
}
