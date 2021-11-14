package org.firstinspires.ftc.teamcode.drive;

public abstract class DriveConstants {
    /*
    public static double TICKS_PER_REV;
    public static double MAX_RPM;
    public static boolean RUN_USING_ENCODER;
    public static PIDFCoefficients MOTOR_VELO_PID;
    public static double WHEEL_RADIUS;
    public static double GEAR_RATIO; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH;
    public static double WHEEL_BASE;
    public static int MAX_VELO;
    public static int SLOW_VELO;
    public static int MAX_ACCEL;
    public static double MAX_ANG_VELO;
    public static double MAX_ANG_ACCEL;

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

    public static DcMotorSimple.Direction[] DIRECTIONS;

    public static DcMotorEx[] driveMotors;
    public static DcMotorEx[] otherMotors;


    public DriveConstants(double TICKS_PER_REV, boolean RUN_USING_ENCODER,
                          PIDFCoefficients MOTOR_VELO_PID, double WHEEL_RADIUS,
                          double GEAR_RATIO, double TRACK_WIDTH, double WHEEL_BASE,
                          int MAX_VELO, int SLOW_VELO, int MAX_ACCEL,
                          double MAX_ANG_VELO, double MAX_ANG_ACCEL, double kV,
                          double kA, double kStatic, DriveConstraints BASE_CONSTRAINTS,
                          PIDCoefficients TRANSLATIONAL_PID, PIDCoefficients HEADING_PID,
                          double LATERAL_MULTIPLIER, double VX_WEIGHT, double VY_WEIGHT,
                          double OMEGA_WEIGHT, int POSE_HISTORY_LIMIT,
                          double slamraX, double slamraY, double MAX_RPM,
                          DcMotorSimple.Direction[] DIRECTIONS){
        this.TICKS_PER_REV = TICKS_PER_REV;
        this.RUN_USING_ENCODER = RUN_USING_ENCODER;
        this.MOTOR_VELO_PID = MOTOR_VELO_PID;
        this.WHEEL_RADIUS = WHEEL_RADIUS;
        this.GEAR_RATIO = GEAR_RATIO;
        this.TRACK_WIDTH = TRACK_WIDTH;
        this.WHEEL_BASE = WHEEL_BASE;
        this.MAX_VELO = MAX_VELO;
        this.SLOW_VELO = SLOW_VELO;
        this.MAX_ACCEL = MAX_ACCEL;
        this.MAX_ANG_VELO = MAX_ANG_VELO;
        this.MAX_ANG_ACCEL = MAX_ANG_ACCEL;
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        this.BASE_CONSTRAINTS = BASE_CONSTRAINTS;
        this.TRANSLATIONAL_PID = TRANSLATIONAL_PID;
        this.HEADING_PID = HEADING_PID;
        this.LATERAL_MULTIPLIER = LATERAL_MULTIPLIER;
        this.VX_WEIGHT = VX_WEIGHT;
        this.VY_WEIGHT = VY_WEIGHT;
        this.OMEGA_WEIGHT = OMEGA_WEIGHT;
        this.POSE_HISTORY_LIMIT = POSE_HISTORY_LIMIT;
        this.slamraX = slamraX;
        this.slamraY = slamraY;
        this.DIRECTIONS = DIRECTIONS;
    }


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS_IMPL * 2 * Math.PI * GEAR_RATIO_IMPL * ticks / TICKS_PER_REV_IMPL;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO_IMPL * 2 * Math.PI * WHEEL_RADIUS_IMPL / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }

    public static void init (HardwareMap hardwareMap){

    }


    public abstract void initialize(HardwareMap hardwareMap);

 */
}
