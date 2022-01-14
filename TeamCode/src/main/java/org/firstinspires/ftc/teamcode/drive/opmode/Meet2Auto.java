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
import org.hermitsocialclub.hydra.opmodes.SubmatSetupOp;
import org.hermitsocialclub.hydra.vision.StaccDetecc;
import org.hermitsocialclub.hydra.vision.VisionPipeline;
import org.hermitsocialclub.hydra.vision.VisionSemaphore;
import org.hermitsocialclub.hydra.vision.util.VisionUtils;
import org.hermitsocialclub.telecat.PersistantTelemetry;
import org.hermitsocialclub.tomato.BarcodeDetect;

@Autonomous(name = "Meet2Auto")
                                                                        public class Meet2Auto extends LinearOpMode {

    private BaselineMecanumDrive drive;
    private PersistantTelemetry telemetry;

    Trajectory backUp;
    Trajectory toBlueHub;

    Pose2d blueStart =  new Pose2d(-38,63,m(90));
    Pose2d blueCarousel = new Pose2d(-12,44,m(90));
    private VisionPipeline visionPipeline;
    private VisionSemaphore semaphore;
    private BarcodeDetect barcodeDetect;

    private MotorConfigurationType carouselType;
    private double carouselSpeed = .3;
    private MotorConfigurationType liftType;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new PersistantTelemetry(super.telemetry);
        drive = new BaselineMecanumDrive(hardwareMap,telemetry);

        drive.setPoseEstimate(blueStart);

                                                        backUp = drive.trajectoryBuilder(blueStart, -90)
                .splineToLinearHeading(new Pose2d(-59.1360, 59.1360,m(135)), m(135))
                .build();

        toBlueHub = drive.trajectoryBuilder(backUp.end(),m(-20))
                .splineToLinearHeading(blueCarousel,m(-90))
                .build();
        barcodeDetect = new BarcodeDetect(true);
        this.semaphore = new VisionSemaphore();
        this.visionPipeline = new VisionPipeline(hardwareMap, telemetry, barcodeDetect, semaphore);
        CameraStreamSource cameraStream = visionPipeline.getCamera();
        FtcDashboard.getInstance().startCameraStream(cameraStream,0);

        carouselType = drive.duck_wheel.getMotorType();

        liftType = drive.lift.getMotorType();

        drive.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.duck_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();


            drive.followTrajectory(backUp);
        drive.duck_wheel.setPower(0.3);
        sleep(1200);
            drive.duck_wheel.setPower(0);
            drive.followTrajectory(toBlueHub);
        drive.lift.setVelocity(liftType
                .getMaxRPM() / 60 * liftType.getAchieveableMaxRPMFraction() * .85 *
                1, AngleUnit.RADIANS);
        sleep(1800);
        drive.outtakeArm.setPosition(.0);
        drive.lift.setPower(0.1);
        sleep(400);
        drive.lift.setPower(0);
        drive.outtakeArm.setPosition(.45);



    }
}
