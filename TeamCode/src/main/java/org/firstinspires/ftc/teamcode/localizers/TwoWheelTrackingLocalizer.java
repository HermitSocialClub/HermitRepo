package org.firstinspires.ftc.teamcode.localizers;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.BaselineMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.hermitsocialclub.telecat.PersistantTelemetry;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->
 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = -1.25; // X is the up and down direction
    public static double PARALLEL_Y = -8; // Y is the strafe direction

    public static double PERPENDICULAR_X = -6.5;
    public static double PERPENDICULAR_Y = -5;

    public static double X_MULTIPLIER = 1.13; // MULTIPLIER IN X DIRECTION
    public static double Y_MULTIPLIER = (99.5/59.5)*0.71494927132117931908466912072023;//1.68; // MULTIPLIER IN Y DIRECTION

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;

    private BaselineMecanumDrive drive;

    private PersistantTelemetry telemetry;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, BaselineMecanumDrive drive, PersistantTelemetry telemetry) {
        super(Arrays.asList(
            new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
            new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;
        this.telemetry = telemetry;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "right_drive_2"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "midEncoder"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        parallelEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> a = Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
        //telemetry.setDebug("Parallel Encoder", a.get(0));
        //telemetry.setDebug("Perpendicular Encoder",a.get(1));

        return a;
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}