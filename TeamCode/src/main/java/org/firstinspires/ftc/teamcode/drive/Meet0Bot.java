package org.firstinspires.ftc.teamcode.drive;

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
public class Meet0Bot extends DriveConstants {

    /*
     * These are motor constants that should be listed online for your motors.
     */
    public static double TICKS_PER_REV_IMPL = 537.6;
    public static final double MAX_RPM_IMPL = 340;

    /*
     * Set the first flag appropriately. If using the built-in motor velocity PID, update
     * MOTOR_VELO_PID with the tuned coefficients from DriveVelocityPIDTuner.
     */
    public static final boolean RUN_USING_ENCODER_IMPL = false;
    public static PIDFCoefficients MOTOR_VELO_PID_IMPL = new PIDFCoefficients(0,0,0, 22.259453425873854);//getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public static double WHEEL_RADIUS_IMPL = 1.9685;
    public static double GEAR_RATIO_IMPL = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH_IMPL = 10.15;
    public static double WHEEL_BASE_IMPL = 13.25;

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */


    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. The velocity and
     * acceleration values are required, and the jerk values are optional (setting a jerk of 0.0
     * forces acceleration-limited profiling).
     */
    public static int MAX_VELO_IMPL = 32;
    public static int SLOW_VELO_IMPL = 20;
    public static int MAX_ACCEL_IMPL = 30;
    public static double MAX_ANG_VELO_IMPL = 3402.608;
    public static double MAX_ANG_ACCEL_IMPL = Math.toRadians(180);

    public static double kV_IMPL = 0.03025;//0.018067801;//.016667801;//1.0 / rpmToVelocity(MAX_RPM);//0.01926780108101678;////0.01587;//0.0135;//
    public static double kA_IMPL = 0.004;//0.0045;//0.003;//0.003;//0.00001;
    public static double kStatic_IMPL = .03;//0.003;//0.02;//0.025;//0.06619;//.05425;

    public static DriveConstraints BASE_CONSTRAINTS_IMPL = new DriveConstraints(
            MAX_VELO, MAX_ACCEL, 0.0,
            MAX_ANG_VELO, MAX_ANG_ACCEL, 0.0
    );
    public static PIDCoefficients TRANSLATIONAL_PID_IMPL = new PIDCoefficients(/*8.3*/8, 8, /*1.1*/.1);
    public static PIDCoefficients HEADING_PID_IMPL = new PIDCoefficients(/*10.4*/8, 1, /*1.5*/0);

    public static double LATERAL_MULTIPLIER_IMPL = 1;//(51.767278876441985/52.5);//(60/45.75) * 1.0434782608695652173913043478261 ;

    public static double VX_WEIGHT_IMPL = 1;
    public static double VY_WEIGHT_IMPL = 1;
    public static double OMEGA_WEIGHT_IMPL = 1;

    public static int POSE_HISTORY_LIMIT_IMPL = 200;

    public static double slamraX_IMPL = 2.75;
    public static double slamraY_IMPL = 2.375;

    public static DcMotorSimple.Direction[] DIRECTIONS_IMPL
            = {DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD,
               DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD};

    public Meet0Bot(){
        super(TICKS_PER_REV_IMPL,RUN_USING_ENCODER_IMPL,MOTOR_VELO_PID_IMPL,
                WHEEL_RADIUS_IMPL,GEAR_RATIO_IMPL,TRACK_WIDTH_IMPL,WHEEL_BASE_IMPL,
                MAX_VELO_IMPL,SLOW_VELO_IMPL,MAX_ACCEL_IMPL,MAX_ANG_VELO_IMPL,
                MAX_ANG_ACCEL,kV_IMPL,kA_IMPL,kStatic_IMPL,BASE_CONSTRAINTS_IMPL,
                TRANSLATIONAL_PID_IMPL,HEADING_PID_IMPL,LATERAL_MULTIPLIER_IMPL,
                VX_WEIGHT_IMPL,VY_WEIGHT_IMPL,OMEGA_WEIGHT_IMPL,
                POSE_HISTORY_LIMIT_IMPL,slamraX_IMPL,slamraY_IMPL,MAX_RPM_IMPL,
                DIRECTIONS_IMPL);
    }

}
