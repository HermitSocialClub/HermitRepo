package org.hermitsocialclub.drive;


import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.hermitsocialclub.localizers.StandardTrackingWheelLocalizer;
import org.hermitsocialclub.util.MoveUtils;
import org.hermitsocialclub.telecat.PersistantTelemetry;
import org.hermitsocialclub.util.DashboardUtil;
import org.hermitsocialclub.util.LynxModuleUtil;
import org.hermitsocialclub.vision.SkystoneVuforiaEngine;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.hermitsocialclub.drive.config.Meet0BotConstants.*;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleMecanumDrive extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(3, 5, 1);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(3, 5, 1);


    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    private PersistantTelemetry telemetry;

    private final FtcDashboard dashboard;
    private final NanoClock clock;

    private Mode mode;

    private final PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    public DriveConstraints constraints;
    private final TrajectoryFollower follower;

    private final List<Pose2d> poseHistory;

    public DcMotorEx leftFront, leftRear, rightRear, rightFront, arm, arm2;
    public Servo topClaw;
    private final List<DcMotorEx> motors;
    private final BNO055IMU imu;
    public ColorSensor colorSensor;
    public HardwareMap hwMap;

    public enum SKYSTONE {
        ONE, TWO, THREE
    }

    public SKYSTONE skystone;

    public SampleMecanumDrive(HardwareMap hardwareMap, SkystoneVuforiaEngine vuforiaEngine) {
        super(kV, kA, kStatic, TRACK_WIDTH, WHEEL_BASE);

        hwMap = hardwareMap;

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        poseHistory = new ArrayList<>();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(DcMotorEx.class, "left_drive");
        leftRear = hardwareMap.get(DcMotorEx.class, "left_drive_2");
        rightRear = hardwareMap.get(DcMotorEx.class, "right_drive_2");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_drive");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm2 = hardwareMap.get(DcMotorEx.class, "arm2");
        topClaw = hardwareMap.get(Servo.class, "topClaw");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            //setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: if desired, use setLocalizer() to change the localization method
        // setLocalizer(new MecanumLocalizerEVI(this,vuforiaEngine,new Pose2d(38,63)));
        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap, telemetry));
    }

    public SampleMecanumDrive(HardwareMap hardwareMap, SkystoneVuforiaEngine vuforiaEngine, PersistantTelemetry telemetry) {
        super(kV, kA, kStatic, TRACK_WIDTH, WHEEL_BASE);

        hwMap = hardwareMap;

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        this.telemetry = telemetry;

        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        poseHistory = new ArrayList<>();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(DcMotorEx.class, "left_drive");
        leftRear = hardwareMap.get(DcMotorEx.class, "left_drive_2");
        rightRear = hardwareMap.get(DcMotorEx.class, "right_drive_2");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_drive");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm2 = hardwareMap.get(DcMotorEx.class, "arm2");
        topClaw = hardwareMap.get(Servo.class, "topClaw");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            //setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: if desired, use setLocalizer() to change the localization method
        // setLocalizer(new MecanumLocalizerEVI(this,vuforiaEngine,new Pose2d(38,63)));
        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap, telemetry));
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, constraints);
    }

    public void turnAsync(double angle) {
        double heading = getPoseEstimate().getHeading();
        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                constraints.maxAngVel,
                constraints.maxAngAccel,
                constraints.maxAngJerk
        );
        turnStart = clock.seconds();
        mode = Mode.TURN;
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void update() {
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        poseHistory.add(currentPose);

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError", lastError.getHeading());

        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(currentPose.getHeading());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose));

                Trajectory trajectory = follower.getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);
                DashboardUtil.drawRobot(fieldOverlay, currentPose);

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

        dashboard.sendTelemetryPacket(packet);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, getMotorVelocityF(8192)
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    Integer linearCPR = 28; //counts per rotation
    Integer LinearGearRatio = 20; //NeverRest 20
    Double linearDiameter = 2.0;
    Double LinearCPI = (linearCPR * 20) / (linearDiameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))

    public void Linear(double Linear_Position, double inches) {

        int move = (int) (Math.round(inches * LinearCPI / 2));

        //robot.arm2.setTargetPosition(robot.arm2.getCurrentPosition() + move);
        arm.setTargetPosition(arm.getCurrentPosition() - move);


        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm.setPower(-Linear_Position);
        arm2.setPower(Linear_Position);

        while (MoveUtils.areAllMotorsBusy(new DcMotor[]{arm})) {
            arm.setPower(-Linear_Position);
            arm2.setPower(Linear_Position);
        }
        arm.setPower(0);
        arm2.setPower(0);
    }

    public void LinearSync(double Linear_Position, double inches) {

        int move = (int) (Math.round(inches * LinearCPI / 2));

        //robot.arm2.setTargetPosition(robot.arm2.getCurrentPosition() + move);
        arm.setTargetPosition(arm.getCurrentPosition() - move);


        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm.setPower(-Linear_Position);
        arm2.setPower(Linear_Position);

        while (MoveUtils.areAllMotorsBusy(new DcMotor[]{arm})) {
            arm.setPower(-Linear_Position);
            arm2.setPower(Linear_Position);
            update();

        }
        arm.setPower(0);
        arm2.setPower(0);
        while (isBusy()) {
            update();
        }

    }

    public void LinearTime(double Linear_Position, double time) {

        ElapsedTime armTime = new ElapsedTime();

        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm.setPower(-Linear_Position);
        arm2.setPower(Linear_Position);

        while (armTime.seconds() < time) {
            arm.setPower(-Linear_Position);
            arm2.setPower(Linear_Position);
        }
        arm.setPower(0);
        arm2.setPower(0);
    }

    public void skystoneDetect() {
        colorSensor = hwMap.get(ColorSensor.class, "Color Sensor");
        while (isBusy() && ((colorSensor.red() * colorSensor.green()) / Math.pow(colorSensor.blue(), 2) >= 3)) {
            if (!isBusy()) {
                return;
            }
        }
        if (getPoseEstimate().getX() > -40) {
            skystone = SKYSTONE.ONE;
        } else if (getPoseEstimate().getX() > -47) {
            skystone = SKYSTONE.TWO;
        } else skystone = SKYSTONE.THREE;
    }
}
