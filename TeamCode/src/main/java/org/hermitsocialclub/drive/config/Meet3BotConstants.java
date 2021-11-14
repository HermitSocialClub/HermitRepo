package org.hermitsocialclub.drive.config;

import com.acmerobotics.dashboard.config.Config;

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
public class Meet3BotConstants extends DriveConstants {
/*

    public static double TICKS_PER_REV = 537.6;
    public static final double MAX_RPM = 340;


    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0,0,0, 22.259453425873854);//getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));


    public static double WHEEL_RADIUS = 3;
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 16.3;
    public static double WHEEL_BASE = 16.5;




    public static int MAX_VELO = 32;
    public static int SLOW_VELO = 20;
    public static int MAX_ACCEL = 30;
    public static double MAX_ANG_VELO = 6;
    public static double MAX_ANG_ACCEL = Math.toRadians(180);

    public static double kV = 0.011962055475993843;//0.03025;//0.018067801;//.016667801;//1.0 / rpmToVelocity(MAX_RPM);//0.01926780108101678;////0.01587;//0.0135;//
    public static double kA = 0.0026;//0.004;//0.0045;//0.003;//0.003;//0.00001;
    public static double kStatic = 0.01;//0.03;//0.003;//0.02;//0.025;//0.06619;//.05425;

    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            MAX_VELO, MAX_ACCEL, 0.0,
            MAX_ANG_VELO, MAX_ANG_ACCEL, 0.0
    );
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0,0,0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0,0,0);

    public static double LATERAL_MULTIPLIER = 1;//(51.767278876441985/52.5);//(60/45.75) * 1.0434782608695652173913043478261 ;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public static int POSE_HISTORY_LIMIT = 200;
    public static double slamraX = 7.5;
    public static double slamraY = -0.5;

    //leftFront, leftRear, rightRear, rightFront
    public static DcMotorSimple.Direction[] DIRECTIONS
            = {DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE};

    public static DcMotorEx[] driveMotors;

    public static DcMotorEx leftFront, leftRear, rightRear, rightFront;
    */
}
