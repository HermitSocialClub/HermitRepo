package org.firstinspires.ftc.teamcode.drive;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.hermitsocialclub.telecat.PersistantTelemetry;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 2048;
    public static double WHEEL_RADIUS = 1; // in
    public static MotorConfigurationType neverRest20GearMotor = MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);
    public static double GEAR_RATIO = neverRest20GearMotor.getGearing(); // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 13; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 9; // in; offset of the lateral wheel

    private PersistantTelemetry telemetry;

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, PersistantTelemetry telemetry) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "tapeShooter"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "arm"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        //leftEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);

        this.telemetry = telemetry;

    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        telemetry.setDebug("leftEncoder Raw Input",leftEncoder.getCurrentPosition());
        telemetry.setDebug("leftEncoder Inches", encoderTicksToInches(leftEncoder.getCurrentPosition()));
        telemetry.setDebug("rightEncoder Raw Input",rightEncoder.getCurrentPosition());
        telemetry.setDebug("rightEncoder Inches", encoderTicksToInches(rightEncoder.getCurrentPosition()));
        telemetry.setDebug("frontEncoder Raw Input",frontEncoder.getCurrentPosition());
        telemetry.setDebug("frontEncoder Inches", encoderTicksToInches(frontEncoder.getCurrentPosition()));

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(frontEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()),
                encoderTicksToInches(frontEncoder.getCorrectedVelocity())
        );
    }
}
