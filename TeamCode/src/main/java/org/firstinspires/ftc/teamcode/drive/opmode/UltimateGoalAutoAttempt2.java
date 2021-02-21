package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.UltimateGoalConfiguration;
import org.hermitsocialclub.hydra.vision.StaccDetecc;
import org.hermitsocialclub.hydra.vision.VisionPipeline;
import org.hermitsocialclub.hydra.vision.VisionSemaphore;
import org.hermitsocialclub.telecat.PersistantTelemetry;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

@Autonomous(name = "Ultimate Goal Zone B Attempt 1")
public class UltimateGoalAutoAttempt2 extends LinearOpMode {

    private enum Zone {
        ZONE_A,
        ZONE_B,
        ZONE_C,
        DEBUG
    }

    private PersistantTelemetry telemetry;
    UltimateGoalConfiguration robot = new UltimateGoalConfiguration();

    private VisionPipeline visionPipeline;
    private StaccDetecc stackDetector;
    private VisionSemaphore semaphore;

    private Zone dropOffZone;
    private Trajectory dropOffPath;

    private DcMotorEx tobeFlywheel;
    private DcMotorEx takeruFlyOut;

    private CRServo kicker;
    private DcMotor wobbleArm;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();
        telemetry = new PersistantTelemetry(super.telemetry);
        BaselineMecanumDrive drive = new BaselineMecanumDrive(hardwareMap, telemetry);
        drive.setPoseEstimate(new Pose2d(-63, -50, Math.toRadians(0)));

        stackDetector = new StaccDetecc();
        semaphore = new VisionSemaphore();
        visionPipeline = new VisionPipeline(hardwareMap, telemetry, stackDetector, semaphore);
        dropOffZone = getZoneFromOpenCV();
        dropOffZone = Zone.DEBUG;

        tobeFlywheel = hardwareMap.get(DcMotorEx.class, "tobeFlywheel");
        kicker = hardwareMap.get(CRServo.class,"kicker");
        takeruFlyOut = hardwareMap.get(DcMotorEx.class,"takeruFlyOut");
        wobbleArm = hardwareMap.get(DcMotor.class, "wobbleArm");
        robot.init(hardwareMap);

        try {
            switch (dropOffZone) {
                case ZONE_A: {
                    dropOffPath = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineToConstantHeading(new Vector2d(24.00, -55.00), 0)
                          //  .addSpatialMarker(new Vector2d(24.00, -55.00), robot.wobbleArm.setPower(1))
                            .splineToConstantHeading(new Vector2d(-3, -30), Math.toRadians(90))
                            .splineToConstantHeading(new Vector2d(-6, 4), Math.toRadians(90))
                            .splineToConstantHeading(new Vector2d(-36, -22), Math.toRadians(200))
                            .splineToSplineHeading(new Pose2d(24, -60, 0), 0)
                            .build();
                    break;
                }
                case ZONE_B: {
                    dropOffPath = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineTo(new Vector2d(-15.00, -50.00), 0)
                            .splineToLinearHeading(new Pose2d(0, -8, 0), Math.toRadians(90))
                            .splineToLinearHeading(new Pose2d(48, -36, 0.00), Math.toRadians(-90))
                            .splineToConstantHeading(new Vector2d(-36, -36), Math.toRadians(90))
                            .splineToConstantHeading(new Vector2d(-20, -36), 0)
                            .splineToConstantHeading(new Vector2d(-40, -26), Math.toRadians(180))
                            .splineToSplineHeading(new Pose2d(20, -36, Math.toRadians(180)), 0)
                            .splineToSplineHeading(new Pose2d(10, -36, Math.toRadians(180)), 0)
                            .build();
                    break;
                }
                case ZONE_C: {
                    dropOffPath = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineToConstantHeading(new Vector2d(0, -36), Math.toRadians(90))
                            .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(90))
                            .splineToSplineHeading(new Pose2d(50, -44, Math.toRadians(90)), Math.toRadians(180))
                            .splineToSplineHeading(new Pose2d( -20, -36, Math.toRadians(180)), Math.toRadians(180))
                            .splineToSplineHeading(new Pose2d(-40, -26, 0), Math.toRadians(180))
                            .splineToSplineHeading(new Pose2d(50, -60, Math.toRadians(180)), 0)
                            .build();
                    break;
                }
                case DEBUG:{
                    dropOffPath = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineToConstantHeading(new Vector2d(-36, -36), Math.toRadians(90))
                            .splineToConstantHeading(new Vector2d(-28,-36),0)
                            .splineToConstantHeading(new Vector2d(40,-36),-30)
                            .splineToConstantHeading(new Vector2d(-36,-28),90)
                            .splineToConstantHeading(new Vector2d(46,-32),-60)
                            .build();
                    break;
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        telemetry.setDebug("Localizer", drive.getLocalizer().toString());
        telemetry.setDebug("Wheel Positions", drive.getWheelPositions());
        telemetry.setDebug("Pose Estimate", "%.2f, %.2f", drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY());
        telemetry.setData("Acceleration", dropOffPath.acceleration(dropOffPath.duration()).toString());
        telemetry.setData("Duration", dropOffPath.duration());
        telemetry.setData("End", dropOffPath.end().toString());
        telemetry.setData("Start", dropOffPath.start().toString());
        telemetry.setData("Path", dropOffPath.getPath().toString());
        telemetry.setData("Velocity", dropOffPath.velocity(dropOffPath.duration()).toString());

        waitForStart();
        if (isStopRequested()) return;

        runtime.reset();
        drive.followTrajectory(dropOffPath);
    }

    private Zone getZoneFromOpenCV() {
        semaphore.waitForFrame();
        int lastStackHeight = stackDetector.getLastStackHeight();
        switch (lastStackHeight) {
            case 0:  return Zone.ZONE_A;
            case 1:  return Zone.ZONE_B;
            case 4:  return Zone.ZONE_C;
            default: throw new IllegalStateException("Detected " + lastStackHeight + " ring stack - this should be impossible!");
        }
    }

}
