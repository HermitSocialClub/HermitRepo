package org.hermitsocialclub.pandemicpanic;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.internal.system.Assert;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.*;

public class MoveUtils {

    public static final double FORWARD = Math.PI / 4;
    public static final double BACKWARD = 5 * Math.PI / 4;
    public static final double LEFT = Math.PI / 2;
    public static final double RIGHT = 2 * Math.PI;
    public static final double SIDE_LEFT = 3 * Math.PI / 4;
    public static final double SIDE_RIGHT = 7 * Math.PI / 4;

    public static final double OVERCLOCK_RATIO = 4 / Math.PI;

    /**
     * <h1>The Algorithm</h1>
     * How it works, no one knows...<br />
     * <br />
     * <a href="https://www.desmos.com/calculator/e6nt1unnkr">Desmos graph of the outputs of the function.</a><br />
     * <a href="https://www.desmos.com/calculator/ggqx6nhvyr">Desmos graph of gamepad x and y to angle.</a><br />
     * <br />     * @param speedModifier Multiplier for final powers.
     * The Algorithm™ is the algorithm that converts an input angle to an output array of motor powers.
     * This implementation returns 4 powers.
     *
     * @param power      Base power for the motors.
     * @param angle      Angle, in radians, for the robot to go in.
     * @param turnOffset Offset for turning.
     * @return an array of 4 doubles, in the form [left_motor, right_motor, left_motor_2, right_motor_2]
     */
    public static double[] theAlgorithm(double power, double angle, double turnOffset, double speedModifier) {
        final double v1 = (power * Math.sin(angle) - turnOffset) * speedModifier;
        final double v2 = (power * Math.cos(angle) + turnOffset) * speedModifier;
        final double v3 = (power * Math.cos(angle) - turnOffset) * speedModifier;
        final double v4 = (power * Math.sin(angle) + turnOffset) * speedModifier;
        return new double[]{v1, v2, v3, v4};
    }

    public static double[] theTurnAlgorithm(double turnOffset) {
        if (turnOffset < 0) return theLeftTurnAlgorithm(Math.abs(turnOffset));
        final double v1 = -turnOffset;
        final double v2 = +turnOffset;
        final double v3 = -turnOffset;
        final double v4 = -turnOffset;
        return new double[]{v1, v2, v3, v4};
    }

    public static double[] theLeftTurnAlgorithm(double turnOffset) {
        final double v1 = +turnOffset;
        final double v2 = -turnOffset;
        final double v3 = +turnOffset;
        final double v4 = -turnOffset;
        return new double[]{v1, v2, v3, v4};
    }

    public static double joystickXYToRadius(double joystickX, double joystickY) {
        return hypot(joystickX, joystickY);  // orig was -X,-Y
    }

    public static double joystickXYToAngle(double joystickX, double joystickY) {
        return atan2(-joystickY, -joystickX) - Math.PI / 4;  // orig was -Y,-X
    }

    public static void setAllMotors(final DcMotor[] motors, final double power) {
        for (int index = 0; index < motors.length; index++) {
            motors[index].setPower(power);
        }
    }

    public static void setEachMotor(final DcMotor[] motors, final double[] powers) {
        Assert.assertTrue(motors.length == powers.length);
        for (int index = 0; index < motors.length; index++) {
            motors[index].setPower(powers[index]);
        }
    }

    public static void useEncoders(final DcMotor[] motors, boolean use) {
        if (use) {
            for (int i = 0; i < motors.length; i++) {
                motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        } else {
            for (int i = 0; i < motors.length; i++) {
                motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }

    public static void resetEncoders(final DcMotor[] motors) {
        for (int i = 0; i < motors.length; i++) {
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public static void runToPosition(final DcMotor[] motors) {
        for (int i = 0; i < motors.length; i++) {
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public static void setEachPosition(final DcMotor[] motors, final Integer[] positions) {
        for (int i = 0; i < motors.length; i++) {
            motors[i].setTargetPosition(positions[i]);
        }
    }

    public static Integer[] getEachPosition(DcMotor[] motors) {
        List<Integer> positions = new ArrayList<>();
        for (int i = 0; i < motors.length; i++) {
            positions.add(motors[i].getCurrentPosition());
        }
        Integer[] posi = new Integer[positions.size()];
        posi = positions.toArray(posi);
        return posi;
    }

    public static boolean areAllMotorsBusy(DcMotor[] motors) {
        boolean allMotorsBusy = false;
        for (int i = 0; i < motors.length; i++) {
            allMotorsBusy = motors[i].isBusy();
        }
        return allMotorsBusy;
    }

    public static boolean areAllMotorsPowered(DcMotor[] motors) {
        boolean allMotorsPowered = false;
        for (int i = 0; i < motors.length; i++) {
            allMotorsPowered = motors[i].getPower() != 0;
        }
        return allMotorsPowered;
    }

    /**
     * Thanks to
     * <a href="https://stackoverflow.com/questions/15022630/how-to-calculate-the-angle-from-rotation-matrix#:~:targetText=If%20R%20is%20the%20(3x3,sum%20of%20the%20diagonal%20elements).">
     * this question answer
     * </a><br />
     * NOTE THAT OPENGLMATRICES ARE 0-INDEXED, UNLIKE NORMAL MATH
     */
    public static float[] poseToRotations(OpenGLMatrix toTarget) {
        double thetaX = atan2(toTarget.get(2, 1), toTarget.get(2, 2));
        double thetaY = atan2(-toTarget.get(2, 0), sqrt(
                pow(toTarget.get(2, 1), 2) + pow(toTarget.get(2, 2), 2)
        ));
        double thetaZ = atan2(toTarget.get(1, 0), toTarget.get(0, 0));
        return new float[]{(float) thetaX, (float) thetaY, (float) thetaZ};
    }

    public static boolean areApproxEqual(double uno, double dos, double threshold) {
        return Math.abs(uno - dos) <= threshold;
    }

    public static double[] multiplyArrays(double[] factor1, double[] factor2) {
        List<Double> product = new ArrayList<>();
        for (int i = 0; i < factor1.length; i++) {
            product.add(factor1[i] * factor2[i]);
        }
        double[] products = new double[product.size()];
        for (int i = 0; i < products.length; i++) {
            products[i] = product.get(i);
        }
        return products;
    }

}
