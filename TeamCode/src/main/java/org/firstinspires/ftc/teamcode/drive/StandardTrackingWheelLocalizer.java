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
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

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

    ExpansionHubMotor motor0, motor1, motor2;
    ExpansionHubEx expansionHub;
    RevBulkData bulkData;



    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, PersistantTelemetry telemetry) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "tapeShooter"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "arm"));

        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        motor0 = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "leftEncoder");
        motor1 = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "tapeShooter");
        motor2 = (ExpansionHubMotor) hardwareMap.get(DcMotorEx.class, "arm");

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
        bulkData = expansionHub.getBulkInputData();
        telemetry.setDebug("leftEncoder Raw Input",leftEncoder.correctRawPosition(bulkData.getMotorCurrentPosition(motor0)));
        telemetry.setDebug("leftEncoder Inches", encoderTicksToInches(leftEncoder.correctRawPosition
                (bulkData.getMotorCurrentPosition(motor0))));
        telemetry.setDebug("rightEncoder Raw Input",rightEncoder.correctRawPosition(bulkData.getMotorCurrentPosition(motor1)));
        telemetry.setDebug("rightEncoder Inches", encoderTicksToInches(rightEncoder.correctRawPosition
                (bulkData.getMotorCurrentPosition(motor1))));
        telemetry.setDebug("frontEncoder Raw Input",frontEncoder.correctRawPosition(bulkData.getMotorCurrentPosition(motor2)));
        telemetry.setDebug("frontEncoder Inches", encoderTicksToInches(frontEncoder.correctRawPosition
                (bulkData.getMotorCurrentPosition(motor2))));

        return Arrays.asList(

                encoderTicksToInches(leftEncoder.correctRawPosition(bulkData.getMotorCurrentPosition(motor0))),
                encoderTicksToInches(rightEncoder.correctRawPosition(bulkData.getMotorCurrentPosition(motor1))),
                encoderTicksToInches(frontEncoder.correctRawPosition(bulkData.getMotorCurrentPosition(motor2)))
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        bulkData = expansionHub.getBulkInputData();

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getTwiceCorrectedVelocity(bulkData.getMotorVelocity(motor0))),
                encoderTicksToInches(rightEncoder.getTwiceCorrectedVelocity(bulkData.getMotorVelocity(motor1))),
                encoderTicksToInches(frontEncoder.getTwiceCorrectedVelocity(bulkData.getMotorVelocity(motor2)))
        );
    }
}
