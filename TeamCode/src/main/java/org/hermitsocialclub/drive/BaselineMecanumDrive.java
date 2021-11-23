package org.firstinspires.ftc.teamcode.drive;


import static org.firstinspires.ftc.teamcode.util.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.DIRECTIONS;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.HEADING_PID;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.LATERAL_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.OMEGA_WEIGHT;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.POSE_HISTORY_LIMIT;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.TRANSLATIONAL_PID;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.VX_WEIGHT;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.VY_WEIGHT;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.encoderTicksToInches;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
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
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.localizers.T265LocalizerRR;
import org.firstinspires.ftc.teamcode.util.DriveConstants;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.hermitsocialclub.telecat.PersistantTelemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class BaselineMecanumDrive extends MecanumDrive {

    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    private FtcDashboard dashboard;
    private NanoClock clock;

    private Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private DriveConstraints constraints;
    private TrajectoryFollower follower;

    private LinkedList<Pose2d> poseHistory;

    public DcMotorEx leftFront, leftRear, rightRear, rightFront, outtake, intake, wobbleArm, lift;
    public List<DcMotorEx> motors;
    private BNO055IMU imu;

    public Servo wobbleGrab;
    public Servo kicker;
    public CRServo hook;
    public CRServo friend;
    public RevColorSensorV3 color;
    public Servo hopperLift;
    public CRServo intakeThirdStage;
   public DcMotorEx duck_wheel;


    private VoltageSensor batteryVoltageSensor;

    private Pose2d lastPoseOnTurn;

    private PersistantTelemetry telemetry;

    public BaselineMecanumDrive(HardwareMap hardwareMap, PersistantTelemetry pt) {



        super(DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        this.telemetry = pt;

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        poseHistory = new LinkedList<>();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        //imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(DcMotorEx.class, "left_drive");
        leftRear = hardwareMap.get(DcMotorEx.class, "left_drive_2");
        rightRear = hardwareMap.get(DcMotorEx.class, "right_drive_2");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_drive");

        lift = hardwareMap.get(DcMotorEx.class,"lift");
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        duck_wheel = hardwareMap.get(DcMotorEx.class,"duck_wheel");


        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            //setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID_IMPL);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        setMotorDirections(DIRECTIONS);


        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        setLocalizer(new T265LocalizerRR(hardwareMap,true));
        telemetry.setData("Pose Estimatlocae",getPoseEstimate());
        telemetry.setData("Pose",T265LocalizerRR.slamra.getLastReceivedCameraUpdate().pose.toString());
        //telemetry.setData("Bot in Use", bot.constants.getClass().toString());
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

        lastPoseOnTurn = getPoseEstimate();

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

        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst();
        }

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

                Pose2d newPose = lastPoseOnTurn.copy(lastPoseOnTurn.getX(), lastPoseOnTurn.getY(), targetState.getX());

                fieldOverlay.setStroke("#4CAF50");
                DashboardUtil.drawRobot(fieldOverlay, newPose);

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
                fieldOverlay.setStroke("#4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, currentPose);

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

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d targetvel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            targetvel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);

        }
        List<Double> powers = MecanumKinematics.robotToWheelVelocities(
                drivePower, 1.0, 1.0, LATERAL_MULTIPLIER);
        telemetry.setData("motor powers",powers.toString());
        setRelativeMotorVelocities(powers);

        telemetry.setData("Wheel Velocities",getWheelVelocities().toString());
        //setDrivePower(vel);
    }

    public void setWeightedDrivePowerPID(Pose2d drivePower) {
        DriveSignal signal;
        PIDFController lateralController = new PIDFController(TRANSLATIONAL_PID);
        PIDFController axialController = new PIDFController(TRANSLATIONAL_PID);
        PIDFController headingController = new PIDFController(HEADING_PID);
        Pose2d curPose = getPoseEstimate();
        telemetry.setData("curPose",curPose);
        Pose2d currentRobotVel = getPoseVelocity();
        telemetry.setData("currentRobotVel",currentRobotVel);
        Pose2d targetPose = new Pose2d(curPose.getX() + drivePower.getX()/10,
                curPose.getY() + drivePower.getY()/10, curPose.getHeading());
        Pose2d targetVel = drivePower;
        Pose2d targetAccel = drivePower.minus(currentRobotVel);
        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            targetVel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
            telemetry.setData("targetVel",targetVel);
            targetPose = new Pose2d(curPose.getX() + drivePower.getX(),
                    curPose.getY() + drivePower.getY(), curPose.getHeading());
            telemetry.setData("Target Pose",targetPose);
            targetAccel = drivePower.minus(getPoseVelocity());
            telemetry.setData("targetAccel",targetAccel);

        }
        Pose2d poseError = Kinematics.calculatePoseError(targetPose, curPose);
        telemetry.setData("poseError",poseError);
        Pose2d targetRobotVel = Kinematics.fieldToRobotVelocity(targetPose, targetVel);
        telemetry.setData("targetRobotVel",targetRobotVel);
        Pose2d targetRobotAccel = Kinematics.fieldToRobotAcceleration(targetPose, targetVel, targetAccel);
        telemetry.setData("targetRobotAccel",targetRobotAccel);

        // you can pass the error directly to PIDFController by setting setpoint = error and measurement = 0
        axialController.setTargetPosition(poseError.getX());
        lateralController.setTargetPosition(poseError.getX());
        headingController.setTargetPosition(poseError.getHeading());

        axialController.setTargetVelocity(targetRobotVel.getX());
        lateralController.setTargetVelocity(targetRobotVel.getY());
        headingController.setTargetVelocity(targetRobotVel.getHeading());

        // note: feedforward is processed at the wheel level
        double axialCorrection = axialController.update(0.0, currentRobotVel.getX());
        telemetry.setData("axialCorrection",axialCorrection);
        double lateralCorrection = lateralController.update(0.0, currentRobotVel.getY());
        telemetry.setData("lateralCorrection",lateralCorrection);
        double headingCorrection = headingController.update(0.0, currentRobotVel.getHeading());
        telemetry.setData("headingCorrection",headingCorrection);

        Pose2d correctedVelocity = targetRobotVel.plus( new Pose2d(
                axialCorrection,
                lateralCorrection,
                headingCorrection
        ));
        telemetry.setData("correctedVelocity",correctedVelocity.toString());

        signal = new DriveSignal(correctedVelocity,targetRobotAccel);

        setDriveSignal(signal);
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

    @Override
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

    public void setRelativeMotorVelocities(List<Double> powers){
        MotorConfigurationType type = MotorConfigurationType.getMotorType(
            NeveRest20Gearmotor.class
    );
        telemetry.setData("RPM Fraction", type.getAchieveableMaxRPMFraction());
        telemetry.setData("Max RPM",type.getMaxRPM());
        int i = 0;
        for (double power:
             powers) {
            motors.get(i)
                    .setVelocity(power * type.getAchieveableMaxRPMFraction()
                    * 2 * Math.PI * type.getMaxRPM()/60,AngleUnit.RADIANS);
            i++;
        }
    }

    public void setMotorDirections (DcMotorSimple.Direction[] directions){

        int i = 0;
        for (DcMotorSimple.Direction direction : directions) {
            motors.get(i).setDirection(direction);
            i++;
        }

    }

    @Override
    public double getRawExternalHeading() {
        return 0;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        // TODO: This must be changed to match your configuration
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // The positive x axis points toward the USB port(s)
        //
        // Adjust the axis rotation rate as necessary
        // Rotate about the z axis is the default assuming your REV Hub/Control Hub is laying
        // flat on a surface

        return (double) imu.getAngularVelocity().zRotationRate;
    }
    public void liftWobble(double position, double power, AngleUnit unit, double timeout){
        ElapsedTime time = new ElapsedTime();
        double finPos = 0;
        double originalPosition = wobbleArm.getCurrentPosition();
        switch (unit){
            case DEGREES: finPos = originalPosition + 90 * position/360; break;
            case RADIANS: finPos = originalPosition + 90 * position/(2*Math.PI); break;
        }
        wobbleArm.setTargetPosition((int) finPos);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArm.setPower(power);
        time.reset();
        while (wobbleArm.isBusy() && time.milliseconds() < timeout){
            telemetry.setDebug("finPos",finPos);
            telemetry.setDebug("currentPosition",wobbleArm.getCurrentPosition());
            telemetry.setDebug("targetPosition",wobbleArm.getTargetPosition());
            telemetry.setDebug("originalPosition",originalPosition);
            telemetry.setDebug("positionDifference",wobbleArm.getTargetPosition() - originalPosition);
        }

        wobbleArm.setPower(0);
        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.setDebug("posError",finPos - wobbleArm.getCurrentPosition());
    }
    public double ticksToRadians(double ticks, MotorConfigurationType motor, int GEAR_RATIO){
        return 2 * Math.PI * GEAR_RATIO * ticks / motor.getTicksPerRev();
    }
}