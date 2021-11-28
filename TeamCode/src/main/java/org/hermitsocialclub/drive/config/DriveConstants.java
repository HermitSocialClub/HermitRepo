package org.hermitsocialclub.drive.config;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * Constants shared between multiple drive types.
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 * fields may also be edited through the dashboard (connect to the robot's WiFi network and
 * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */
@Config
public class DriveConstants {

    /*
     * These are motor constants that should be listed online for your motors.
     */
    public static double TICKS_PER_REV;

    public static double MAX_RPM;

    /*
     * Set the first flag appropriately. If using the built-in motor velocity PID, update
     * MOTOR_VELO_PID with the tuned coefficients from DriveVelocityPIDTuner.
     */

    public static boolean RUN_USING_ENCODER;

    public static PIDFCoefficients MOTOR_VELO_PID;

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public static double WHEEL_RADIUS;

    public static double GEAR_RATIO; // output (wheel) speed / input (motor) speed

    public static double TRACK_WIDTH;

    public static double WHEEL_BASE;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. The velocity and
     * acceleration values are required, and the jerk values are optional (setting a jerk of 0.0
     * forces acceleration-limited profiling).
     */
    public static int MAX_VELO;

    public static int MAX_ACCEL;

    public static double MAX_ANG_VELO;

    public static double MAX_ANG_ACCEL;

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public static double kV;

    public static double kA;

    public static double kStatic;

    public static DriveConstraints BASE_CONSTRAINTS;

    public static PIDCoefficients TRANSLATIONAL_PID;


    public static PIDCoefficients HEADING_PID;


    public static double LATERAL_MULTIPLIER;

    public static double VX_WEIGHT;

    public static double VY_WEIGHT;

    public static double OMEGA_WEIGHT;

    public static int POSE_HISTORY_LIMIT;

    public static double slamraX;

    public static double slamraY;

    //leftFront, leftRear, rightRear, rightFront
    public static DcMotorSimple.Direction[] DIRECTIONS;

    //The Literal Joystick
    /*static {
        TICKS_PER_REV = 537.6;

        MAX_RPM = 340;

        RUN_USING_ENCODER = true;

        MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, 22.259453425873854);

        WHEEL_RADIUS = 1.9685;

        GEAR_RATIO = 1;

        TRACK_WIDTH = 10.15;

        WHEEL_BASE = 13.25;

        MAX_VELO = 32;

        MAX_ACCEL = 30;

        MAX_ANG_VELO = 3402.608;

        MAX_ANG_ACCEL = Math.toRadians(180);

        kV = 0.03025;

        kA = 0.004;

        kStatic = .03;

        BASE_CONSTRAINTS = new DriveConstraints(
            MAX_VELO, MAX_ACCEL, 0.0,
            MAX_ANG_VELO, MAX_ANG_ACCEL, 0.0
        );
        TRANSLATIONAL_PID = new PIDCoefficients(8, 8, .1);

        HEADING_PID = new PIDCoefficients(8, 1, 0);

        LATERAL_MULTIPLIER = 1;

        VX_WEIGHT = 1;

        VY_WEIGHT = 1;

        OMEGA_WEIGHT = 1;

        POSE_HISTORY_LIMIT = 200;

        slamraX = 2.75;

        slamraY = 2.375;

        DIRECTIONS = new DcMotorSimple.Direction[]{DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE};
    }*/

    //Big Bertha
    static {
        TICKS_PER_REV = 537.6;

        MAX_RPM = 340;

        RUN_USING_ENCODER = false;

        MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, 22.259453425873854);

        WHEEL_RADIUS = 3;

        GEAR_RATIO = 1;

        TRACK_WIDTH = 16.3;

        WHEEL_BASE = 16.5;

        MAX_VELO = 32;

        MAX_ACCEL = 30;

        MAX_ANG_VELO = 6;

        MAX_ANG_ACCEL = Math.toRadians(180);

        kV = 0.011962055475993843;

        kA = 0.0026;

        kStatic = 0.01;

        BASE_CONSTRAINTS = new DriveConstraints(
                MAX_VELO, MAX_ACCEL, 0.0,
                MAX_ANG_VELO, MAX_ANG_ACCEL, 0.0
        );
        TRANSLATIONAL_PID = new PIDCoefficients(0,0,0);

        HEADING_PID = new PIDCoefficients(0,0,0);

        LATERAL_MULTIPLIER = 1;

        VX_WEIGHT = 1;

        VY_WEIGHT = 1;

        OMEGA_WEIGHT = 1;

        POSE_HISTORY_LIMIT = 200;

        slamraX = 7.5;

        slamraY = -0.5;

        DIRECTIONS = new DcMotorSimple.Direction[]{DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE};
    }

    //The OSHA Offender
    /*static {
        TICKS_PER_REV = 537.6;

        MAX_RPM = 340;

        RUN_USING_ENCODER = false;

        MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
                getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

        WHEEL_RADIUS = 1.9685;

        GEAR_RATIO = 4.0/3.0;

        TRACK_WIDTH = 6.5;

        WHEEL_BASE = 6.5;

        MAX_VELO = 79;

        MAX_ACCEL = 79;

        MAX_ANG_VELO = Math.toRadians(360);

        MAX_ANG_ACCEL = Math.toRadians(360);

        kV = 1.0 / rpmToVelocity(MAX_RPM);

        kA = 0;

        kStatic = 0;

        BASE_CONSTRAINTS = new DriveConstraints(
                MAX_VELO, MAX_ACCEL, 0.0,
                MAX_ANG_VELO, MAX_ANG_ACCEL, 0.0
        );
        TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);

        HEADING_PID = new PIDCoefficients(0, 0, 0);

        LATERAL_MULTIPLIER = 1;

        VX_WEIGHT = 1;

        VY_WEIGHT = 1;

        OMEGA_WEIGHT = 1;

        POSE_HISTORY_LIMIT = 200;

        slamraX = 1.25;//Robot Length is 15.5 inches, slamra is 6.5 inches from the front

        slamraY = 3.875;//Robot Width is 12.25 inches, slamra is 2.25 inches from the right

        DIRECTIONS = new DcMotorSimple.Direction[]{DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE};
    }*/

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
