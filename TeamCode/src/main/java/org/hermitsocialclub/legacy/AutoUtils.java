package org.hermitsocialclub.legacy;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.hermitsocialclub.pandemicpanic.MecanumConfiguration;
import org.hermitsocialclub.util.MoveUtils;
import org.hermitsocialclub.telecat.PersistantTelemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

import static java.lang.Thread.sleep;

public class AutoUtils {

    public Thread gyroCheck;
    public Thread encoderCheck;
    public AtomicBoolean gyroPause = new AtomicBoolean(false);
    private Telemetry t;
    public static final int cpr = 28; //counts per rotation
    public static final int gearratio = 20;
    public static final double diameter = 4.125;
    public static final double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    public static final double cpmm = cpi * 25.4; // converts cpi to mm for vuforia
    public static final double bias = 1.0;//default 0.8
    public static final double meccyBias = 0.95;//change to adjust only strafing movement
    public static final double conversion = cpi * bias;

    public static final double angleThreshold = 5;

    public double initialAngle = 0;

    private final PersistantTelemetry telemetry;
    private final MecanumConfiguration robot;

    Supplier<AtomicInteger> left1Encoder;
    Supplier<AtomicInteger> left2Encoder;
    Supplier<AtomicInteger> right1Encoder;
    Supplier<AtomicInteger> right2Encoder;

    public volatile int[] encoders;

    private final ElapsedTime runtime;
    private final Supplier<Boolean> opModeIsActive;
    public boolean collisonDetected = false;
    public boolean exit = false;
    public SkystoneNumber whichBlockIsTheSkystone;
    public double collisionThreshholdX = .1;
    public double collisionThreshholdY = .1;
    public Side startSide = Side.RED;
    public double collisionThreshholdEncoders = 1;
    Orientation angles;
    Acceleration gravity;

    public enum Direction {
        FORWARDS, BACKWARDS, LEFT, RIGHT
    }

    public enum Color {
        RED, GREEN, BLUE, YELLOW, BLACK, WHITE
    }

    public Direction currentDirection;

    public enum SkystoneNumber {

        ONE(-98, -60, -1, -1.5, 1500),
        TWO(-86.5, -114, 8, 6.5, 1500),
        THREE(-105, -80, 15, 14.5, 1800);
        public double distance;
        public double returnBlock;
        public double secondBlockX;
        public double secondBlockY;
        public double linearDistance;

        SkystoneNumber(double distance, double returnBlock, double secondBlockX, double secondBlockY, double linearDistance) {
            this.distance = distance;
            this.returnBlock = returnBlock;
            this.secondBlockX = secondBlockX;
            this.secondBlockY = secondBlockY;
            this.linearDistance = linearDistance;
        }

    }

    public enum Side {
        RED, BLUE
    }

    public AutoUtils(PersistantTelemetry telemetry, MecanumConfiguration robot, Supplier<Boolean> opModeIsActive, ElapsedTime runtime,
                     Supplier<AtomicInteger> left1Encoder, Supplier<AtomicInteger> right1Encoder, Supplier<AtomicInteger> left2Encoder,
                     Supplier<AtomicInteger> right2Encoder) {
        this.telemetry = telemetry;
        this.robot = robot;
        this.opModeIsActive = opModeIsActive;
        this.runtime = runtime;
        this.left1Encoder = left1Encoder;
        this.left2Encoder = left2Encoder;
        this.right1Encoder = right1Encoder;
        this.right2Encoder = right2Encoder;
    }

    public void moveToPosition(double inches, double speed) {
        int move = (int) (Math.round(inches * conversion));

        robot.left_drive_2.setTargetPosition(robot.left_drive_2.getCurrentPosition() + move);
        robot.left_drive.setTargetPosition(robot.left_drive.getCurrentPosition() + move);
        robot.right_drive_2.setTargetPosition(robot.right_drive_2.getCurrentPosition() + move);
        robot.right_drive.setTargetPosition(robot.right_drive.getCurrentPosition() + move);

        robot.left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.left_drive.setPower(speed);
        robot.left_drive_2.setPower(speed);
        robot.right_drive.setPower(speed);
        robot.right_drive_2.setPower(speed);

        while (robot.left_drive.isBusy() && robot.right_drive.isBusy() && robot.left_drive_2.isBusy() && robot.right_drive_2.isBusy()) {
            if (exit) {
                robot.right_drive.setPower(0);
                robot.left_drive.setPower(0);
                robot.right_drive_2.setPower(0);
                robot.left_drive_2.setPower(0);
                return;
            }
        }
        robot.right_drive.setPower(0);
        robot.left_drive.setPower(0);
        robot.right_drive_2.setPower(0);
        robot.left_drive_2.setPower(0);
        return;
    }

    /*
    This function uses the Expansion Hub IMU Integrated Gyro to turn a precise number of degrees (+/- 5).
    Degrees should always be positive, make speedDirection negative to turn left.
     */
    public void turnWithGyro(double degrees, double speedDirection) {

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle; //make this negative
        telemetry.setData("Speed Direction", speedDirection);
        telemetry.setData("Yaw", yaw);

        telemetry.setData("stuff", speedDirection);

        double first;
        double second;

        if (speedDirection > 0) {

            if (degrees > 10) {
                first = (degrees - 10) + devert(yaw);
                second = degrees + devert(yaw);
            } else {
                first = devert(yaw);
                second = degrees + devert(yaw);
            }

        } else {

            if (degrees > 10) {
                first = devert(-(degrees - 10) + devert(yaw));
                second = devert(-degrees + devert(yaw));
            } else {
                first = devert(yaw);
                second = devert(-degrees + devert(yaw));
            }

        }

        Double firsta = convert(first - 5); // 175
        Double firstb = convert(first + 5); //-175

        turnWithEncoder(speedDirection);

        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive.get()) { //within range?
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = robot.imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.setData("Position", yaw);
                telemetry.setData("first before", first);
                telemetry.setData("first after", convert(first));

            }
        } else {

            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive.get()) {//within range?
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = robot.imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.setData("Position", yaw);
                telemetry.setData("first before", first);
                telemetry.setData("first after", convert(first));

            }
        }

        Double seconda = convert(second - 5);//175
        Double secondb = convert(second + 5);//-175

        turnWithEncoder(speedDirection / 3);

        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive.get()) { //within range?
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = robot.imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.setData("Position", yaw);
                telemetry.setData("second before", second);
                telemetry.setData("second after", convert(second));

            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive.get()) { //within range?
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = robot.imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.setData("Position", yaw);
                telemetry.setData("second before", second);
                telemetry.setData("second after", convert(second));

            }
            robot.left_drive.setPower(0);
            robot.right_drive.setPower(0);
            robot.left_drive_2.setPower(0);
            robot.right_drive_2.setPower(0);
        }


        robot.left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_drive_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_drive_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    /*
    This function uses the encoders to strafe left or right.
    Negative input for inches results in right strafing.
     */
    public void strafeToPosition(double inches, double speed) {

        int move = (int) (Math.round(inches * cpi * meccyBias));

        robot.left_drive_2.setTargetPosition(robot.left_drive_2.getCurrentPosition() - move);
        robot.left_drive.setTargetPosition(robot.left_drive.getCurrentPosition() + move);
        robot.right_drive_2.setTargetPosition(robot.right_drive_2.getCurrentPosition() + move);
        robot.right_drive.setTargetPosition(robot.right_drive.getCurrentPosition() - move);

        robot.left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.left_drive.setPower(1.1 * speed);
        robot.left_drive_2.setPower(speed);
        robot.right_drive.setPower(1.1 * speed);
        robot.right_drive_2.setPower(speed);

        while (robot.left_drive.isBusy() && robot.right_drive.isBusy() && robot.left_drive_2.isBusy() && robot./*Hello General Kenobi*/right_drive_2.isBusy()) {

        }
        robot.right_drive.setPower(0);
        robot.left_drive.setPower(0);
        robot.right_drive_2.setPower(0);
        robot.left_drive_2.setPower(0);
        return;
    }

    /*
    These functions are used in the turnWithGyro function to ensure inputs
    are interpreted properly.
     */
    public double devert(double degrees) {
        if (degrees < 0) {
            degrees = degrees + 360;
        }
        return degrees;
    }

    public double convert(double degrees) {
        if (degrees > 179) {
            degrees = -(360 - degrees);
        } else if (degrees < -180) {
            degrees = 360 + degrees;
        } else if (degrees > 360) {
            degrees = degrees - 360;
        }
        return degrees;
    }

    /*
    This function is used in the turnWithGyro function to set the
    encoder mode and turn.
     */
    public void turnWithEncoder(double input) {
        robot.left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_drive_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_drive_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        robot.left_drive.setPower(input);
        robot.left_drive_2.setPower(input);
        robot.right_drive.setPower(-input);
        robot.right_drive_2.setPower(-input);
    }

    /*
     * This function is used to move while the color sensor is detecting color
     */
    // Move_With_Color has been removed temporarily
    public void move_With_Color(Color color, Direction direction,
                                double speed, double inches, double timeout, Side side) {
        runtime.reset();
        robot.left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_drive_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_drive_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double initialposition = robot.left_drive.getCurrentPosition();
        telemetry.setData("initialPosition", initialposition);
        boolean finished = false;
        while (!exit && !finished) {
            switch (direction) {
                case LEFT: {
                    int move = (int) (Math.round(inches * cpi * meccyBias));

                    robot.left_drive_2.setTargetPosition(robot.left_drive_2.getCurrentPosition() + move);
                    robot.left_drive.setTargetPosition(robot.left_drive.getCurrentPosition() - move);
                    robot.right_drive_2.setTargetPosition(robot.right_drive_2.getCurrentPosition() - move);
                    robot.right_drive.setTargetPosition(robot.right_drive.getCurrentPosition() + move);

                    robot.left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.left_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.right_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    robot.left_drive.setPower(1.1 * speed);
                    robot.left_drive_2.setPower(speed);
                    robot.right_drive.setPower(1.1 * speed);
                    robot.right_drive_2.setPower(speed);
                    break;
                }
                case RIGHT: {
                    int move = (int) (Math.round(inches * cpi * meccyBias));

                    robot.left_drive_2.setTargetPosition(robot.left_drive_2.getCurrentPosition() - move);
                    robot.left_drive.setTargetPosition(robot.left_drive.getCurrentPosition() + move);
                    robot.right_drive_2.setTargetPosition(robot.right_drive_2.getCurrentPosition() + move);
                    robot.right_drive.setTargetPosition(robot.right_drive.getCurrentPosition() - move);

                    robot.left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.left_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.right_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    robot.left_drive.setPower(1.1 * speed);
                    robot.left_drive_2.setPower(speed);
                    robot.right_drive.setPower(1.1 * speed);
                    robot.right_drive_2.setPower(speed);
                    break;
                }
                case FORWARDS: {
                    int move = (int) (Math.round(inches * conversion));

                    robot.left_drive_2.setTargetPosition(robot.left_drive_2.getCurrentPosition() + move);
                    robot.left_drive.setTargetPosition(robot.left_drive.getCurrentPosition() + move);
                    robot.right_drive_2.setTargetPosition(robot.right_drive_2.getCurrentPosition() + move);
                    robot.right_drive.setTargetPosition(robot.right_drive.getCurrentPosition() + move);

                    robot.left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.left_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.right_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    robot.left_drive.setPower(speed);
                    robot.left_drive_2.setPower(speed);
                    robot.right_drive.setPower(speed);
                    robot.right_drive_2.setPower(speed);
                    break;
                }
                case BACKWARDS: {
                    int move = (int) (Math.round(inches * conversion));

                    robot.left_drive_2.setTargetPosition(robot.left_drive_2.getCurrentPosition() - move);
                    robot.left_drive.setTargetPosition(robot.left_drive.getCurrentPosition() - move);
                    robot.right_drive_2.setTargetPosition(robot.right_drive_2.getCurrentPosition() - move);
                    robot.right_drive.setTargetPosition(robot.right_drive.getCurrentPosition() - move);

                    robot.left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.left_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.right_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    robot.left_drive.setPower(speed);
                    robot.left_drive_2.setPower(speed);
                    robot.right_drive.setPower(speed);
                    robot.right_drive_2.setPower(speed);
                    break;
                }
            }
            // Detects skystone by assuming yellow until the condition value for RGB falls below 3
            switch (color) {
                case YELLOW: {
                    while (((robot.left_drive.isBusy() && robot.right_drive.isBusy()
                            && robot.left_drive_2.isBusy() && robot.right_drive_2.isBusy())
                            && ((robot.colorSensor.red() * robot.colorSensor.green()) / Math.pow(robot.colorSensor.blue(), 2) >= 3)
                            && runtime.milliseconds() <= timeout)) {
                        telemetry.setData("Saw Skystone", (robot.colorSensor.red() * robot.colorSensor.green()) /
                                Math.pow(robot.colorSensor.blue(), 2) >= 3);
                        telemetry.setData("Distance Travelled", robot.left_drive.getCurrentPosition());
                        telemetry.setData("RGB Values",
                                "R: %d, G: %d, B: %d",
                                robot.colorSensor.red(), robot.colorSensor.green(), robot.colorSensor.blue());
                        //telemetry.setData("Pos","X: %f, Y: %f",vuforiaEngine.getPos()[0],vuforiaEngine.getPos()[1]);

                        if (exit) {
                            robot.right_drive.setPower(0);
                            robot.left_drive.setPower(0);
                            robot.right_drive_2.setPower(0);
                            robot.left_drive_2.setPower(0);
                            return;
                        }
                    }
                    robot.left_drive.setPower(0);
                    robot.right_drive.setPower(0);
                    robot.left_drive_2.setPower(0);
                    robot.right_drive_2.setPower(0);
                    break;
                }
            }
            /*stores skystone position value based on distance traveled. length of stones are 8in
            so assumes <8in is ONE, >8 and <16 is TWO and >16 is THREE.
            PROBLEM - this can fail depending on where BOT starts scanning.  is there a better
            solution?
             */
            if (side == Side.BLUE) {
                if (Math.abs(robot.left_drive.getCurrentPosition() - initialposition) / cpi <= 8) {
                    whichBlockIsTheSkystone = SkystoneNumber.ONE;
                } else if (Math.abs(robot.left_drive.getCurrentPosition() - initialposition) / cpi >= 8 && Math.abs(robot.left_drive.getCurrentPosition() - initialposition) / cpi <= 19) {
                    whichBlockIsTheSkystone = SkystoneNumber.TWO;
                } else {
                    whichBlockIsTheSkystone = SkystoneNumber.THREE;
                }
                telemetry.setData("Distance to Come", whichBlockIsTheSkystone.distance);
                finished = true;
            } else {
                if (Math.abs(robot.left_drive.getCurrentPosition() - initialposition) / cpi <= 7) {
                    whichBlockIsTheSkystone = SkystoneNumber.ONE;
                } else if (Math.abs(robot.left_drive.getCurrentPosition() - initialposition) / cpi >= 7 && Math.abs(robot.left_drive.getCurrentPosition() - initialposition) / cpi <= 12) {
                    whichBlockIsTheSkystone = SkystoneNumber.TWO;
                } else {
                    whichBlockIsTheSkystone = SkystoneNumber.THREE;
                }
                telemetry.setData("Distance to Come", whichBlockIsTheSkystone.distance);
                finished = true;
            }
            strafeToPosition(5.5, .4);
            moveToPosition(1.5, .25);
            telemetry.setData("Distance Traveled", (robot.left_drive.getCurrentPosition() - initialposition) / cpi);

        }
        MoveUtils.setAllMotors(robot.drive_Motors, 0);
        return;
    }

    public void clampinator(double position) {
        robot.block_Clamper.setPosition(position);
        robot.block_Clamper_2.setPosition(Math.abs(position - 1));
    }

    public void TurnRight(long degrees, double speed) {
        Orientation orientation = robot.imu.getAngularOrientation();
        double xangle = orientation.firstAngle;
        double initialangle = xangle;

        while (xangle - initialangle <= degrees) {
            telemetry.setData("Right turn", "Yes");
            MoveUtils.setEachMotor(robot.drive_Motors, new double[]{-speed, -speed, speed, speed});
            orientation = robot.imu.getAngularOrientation();
            xangle = orientation.firstAngle;
            telemetry.setData("Degrees", xangle);
            telemetry.setData("Initial Position", initialangle);
            telemetry.setData("Change in Degrees", xangle - initialangle);
        }
        MoveUtils.setAllMotors(robot.drive_Motors, 0);


    }

    public void collisionStrafe(double inches, double speed) {
        ElapsedTime cycleTime = new ElapsedTime();

        int move = (int) (Math.round(inches * cpi * meccyBias));

        robot.left_drive_2.setTargetPosition(robot.left_drive_2.getCurrentPosition() - move);
        robot.left_drive.setTargetPosition(robot.left_drive.getCurrentPosition() + move);
        robot.right_drive_2.setTargetPosition(robot.right_drive_2.getCurrentPosition() + move);
        robot.right_drive.setTargetPosition(robot.right_drive.getCurrentPosition() - move);

        robot.left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.left_drive.setPower(speed);
        robot.left_drive_2.setPower(speed);
        robot.right_drive.setPower(speed);
        robot.right_drive_2.setPower(speed);

        Velocity acceleration = robot.imu.getVelocity();
        double lastXAccel = acceleration.xVeloc;
        double lastYAccel = acceleration.yVeloc;
        double currXAccel;
        double currYAccel;
        double jerkX;
        double jerkY;

        List<Integer> lastEncoders = new ArrayList<>();
        List<Integer> currentEncoders = new ArrayList<>();
        List<Integer> encoderDifferences = new ArrayList<>();
        int encoderDifferenceAverage = 0;
        double encoderDifferenceAverageOverTime = 0;
        for (int i = 0; i < robot.drive_Motors.length; i++) {
            lastEncoders.add(robot.drive_Motors[i].getCurrentPosition());
        }

        while (robot.left_drive.isBusy() && robot.right_drive.isBusy() && robot.left_drive_2.isBusy() && robot.right_drive_2.isBusy()) {
            currXAccel = acceleration.xVeloc;
            jerkX = currXAccel - lastXAccel;
            currYAccel = acceleration.yVeloc;
            jerkY = currYAccel - lastYAccel;

            telemetry.setData("X Accel", acceleration.xVeloc);
            telemetry.setData("Y Accel", acceleration.yVeloc);
            telemetry.setData("Last X Accel", lastXAccel);
            telemetry.setData("Last Y Accel", lastYAccel);
            telemetry.setData("X Jerk", jerkX);
            telemetry.setData("Y Jerk", jerkY);
            if (Math.abs(jerkX) > 0) {
                lastXAccel = currXAccel;
            }
            if (Math.abs(jerkY) > 0) {
                lastYAccel = currYAccel;
            }
            for (int i = 0; i < robot.drive_Motors.length; i++) {
                currentEncoders.add(robot.drive_Motors[i].getCurrentPosition());
            }
            for (int i = 0; i < currentEncoders.size(); i++) {
                encoderDifferences.add(currentEncoders.get(i) - lastEncoders.get(i));
            }
            lastEncoders.clear();
            for (int i = 0; i < currentEncoders.size(); i++) {
                lastEncoders.add(currentEncoders.get(i));
            }
            currentEncoders.clear();
            for (int i = 0; i < encoderDifferences.size(); i++) {
                encoderDifferenceAverage += encoderDifferences.get(i);
            }
            encoderDifferenceAverage /= encoderDifferences.size();
            encoderDifferenceAverageOverTime = encoderDifferenceAverage / (cycleTime.milliseconds() / 1000);
            telemetry.setData("Encoder Difference Average", encoderDifferenceAverage);
            if (Math.abs(jerkX) > collisionThreshholdX || Math.abs(jerkY) > collisionThreshholdY
                    || Math.abs(encoderDifferenceAverageOverTime) > collisionThreshholdEncoders) {
                while (opModeIsActive.get()) {
                    telemetry.setData("encoderDifferenceAverageOverTime", encoderDifferenceAverageOverTime);
                    if (Math.abs(jerkX) > collisionThreshholdX || Math.abs(jerkY) > collisionThreshholdY || encoderDifferenceAverageOverTime > collisionThreshholdX) {
                        while (opModeIsActive.get()) {
                            MoveUtils.setAllMotors(robot.drive_Motors, 0);
                            telemetry.setData("Collided", "%f,%f", jerkX, jerkY);
                        }
                        collisonDetected = true;
                        return;
                    }
                    encoderDifferenceAverage = 0;
                    encoderDifferences.clear();
                    cycleTime.reset();
                }
                robot.right_drive.setPower(0);
                robot.left_drive.setPower(0);
                robot.right_drive_2.setPower(0);
                robot.left_drive_2.setPower(0);
                return;
            }
        }
    }

    public void TurnRight2(long degrees, double speed) {
        Orientation orientation;
        orientation = robot.imu.getAngularOrientation();
        double xangle = orientation.firstAngle;
        double initialangle = xangle;

        while (xangle - initialangle <= degrees) {
            telemetry.setData("Right turn", "Yes");
            MoveUtils.setEachMotor(robot.drive_Motors, new double[]{-speed, -speed, speed, speed});
            orientation = robot.imu.getAngularOrientation();
            xangle = orientation.firstAngle;
            telemetry.setData("Degrees", xangle);
            telemetry.setData("Initial Position", initialangle);
            telemetry.setData("Change in Degrees", xangle - initialangle);
        }
        MoveUtils.setAllMotors(robot.drive_Motors, 0);
    }

    public void gyroChecker() {
        gyroCheck = new Thread(() -> {
            try {
                Orientation orientation = robot.imu.getAngularOrientation();
                double angle;
                double initialAngle = orientation.firstAngle;
                while (!exit) {
                    angle = orientation.firstAngle - initialAngle;
                    if (angle > angleThreshold) {
                        gyroPause.set(true);
                        TurnRight((long) angle, .2);
                    }
                    telemetry.setData("Angle Elapsed", angle);
                    telemetry.setData("Pause for Gyro Adjustment?", gyroPause.get());
                }
            } catch (Exception e) {
                throw new RuntimeException("Thread " +
                        "interrupted");
            }
        });
        gyroCheck.start();
    }

    public void gyroSyncStrafe(double inches, double speed) {

        MoveUtils.resetEncoders(robot.drive_Motors);

        Orientation o = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double currentangle;
        double lastAngle;

        double moveAngle;

        double[] speeds;
        double[] lastSpeeds;

        ElapsedTime cycleTime = new ElapsedTime();
        ElapsedTime updateTime = new ElapsedTime();

        if (inches > 0) {
            currentDirection = Direction.RIGHT;
        } else currentDirection = Direction.LEFT;

        double P = .2;
        double I = 0.2;
        double D = 0.20;
        double correction = 0;

        double error = 0;
        double integral = 0;
        double derivative = 0;
        double lastError = 0;

        int move = (int) (Math.round(inches * cpi * meccyBias));

        robot.left_drive_2.setTargetPosition(robot.left_drive_2.getCurrentPosition() - move);
        robot.left_drive.setTargetPosition(robot.left_drive.getCurrentPosition() + move);
        robot.right_drive_2.setTargetPosition(robot.right_drive_2.getCurrentPosition() + move);
        robot.right_drive.setTargetPosition(robot.right_drive.getCurrentPosition() - move);
        //
        robot.left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        robot.left_drive.setPower(speed);
        robot.left_drive_2.setPower(speed);
        robot.right_drive.setPower(speed);
        robot.right_drive_2.setPower(speed);


        telemetry.setData("Initial Angle", initialAngle);


        while (MoveUtils.areAllMotorsBusy(robot.drive_Motors)) {
            o = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            currentangle = o.firstAngle;

            telemetry.setData("Current Angle", currentangle);

            error = initialAngle - currentangle;
            integral = (integral / 3) + error * cycleTime.seconds();
            derivative = (error - lastError) / cycleTime.seconds();

            telemetry.setData("Error", error);
            telemetry.setData("Integral", integral);
            telemetry.setData("Derivative", derivative);

            correction = P * error + I * integral + D * derivative;

            MoveUtils.setEachMotor(robot.drive_Motors, new double[]{speed - correction, speed + correction,
                    speed - correction, speed + correction});

            if (error != lastError) {
                telemetry.setData("Update Time", updateTime.milliseconds());
                updateTime.reset();
            }

            if (!MoveUtils.areAllMotorsBusy(robot.drive_Motors) || !MoveUtils.areAllMotorsPowered(robot.drive_Motors)) {
                telemetry.setData("Stopping the Robot", "");
                MoveUtils.setAllMotors(robot.drive_Motors, 0);
                return;
            }

            lastError = error;
            telemetry.setData("cycle time", cycleTime.milliseconds());
            cycleTime.reset();

        }
        robot.right_drive.setPower(0);
        robot.left_drive.setPower(0);
        robot.right_drive_2.setPower(0);
        robot.left_drive_2.setPower(0);
        return;
    }

    public void syncStrafe(double inches, double speed) {

        if (inches > 0) {
            currentDirection = Direction.RIGHT;
        } else currentDirection = Direction.LEFT;
        //
        int move = (int) (Math.round(inches * cpi * meccyBias));
        //

        PID pid = new PID(1.4, 1.4, .8);
        pid.setOutputFilter(.2);

        double[] integral = {0, 0, 0};
        double[] derivative = {0, 0, 0};
        double[] lastError = {0, 0, 0};
        double[] finalPid = {0, 0, 0};

        double referenceSpeed;
        double lastReferenceEncoder = robot.right_drive_2.getCurrentPosition();
        double referenceEncoder = robot.right_drive_2.getCurrentPosition();
        double referenceEncoderDifference;


        ElapsedTime referenceCycleTime = new ElapsedTime();
        ElapsedTime left1CycleTime = new ElapsedTime();
        ElapsedTime right1CycleTime = new ElapsedTime();
        ElapsedTime left2CycleTime = new ElapsedTime();
        ElapsedTime[] motorCycleTimes = {left1CycleTime, right1CycleTime, left2CycleTime};
        int[] currentEncoders = new int[]{robot.left_drive.getCurrentPosition(), robot.right_drive.getCurrentPosition(),
                robot.left_drive_2.getCurrentPosition()};
        int[] lastEncoders = {robot.left_drive.getCurrentPosition(), robot.right_drive.getCurrentPosition(),
                robot.left_drive_2.getCurrentPosition()};

        robot.left_drive_2.setTargetPosition(robot.left_drive_2.getCurrentPosition() - move);
        robot.left_drive.setTargetPosition(robot.left_drive.getCurrentPosition() + move);
        robot.right_drive_2.setTargetPosition(robot.right_drive_2.getCurrentPosition() + move);
        robot.right_drive.setTargetPosition(robot.right_drive.getCurrentPosition() - move);
        //
        robot.left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        robot.left_drive.setPower(speed);
        robot.left_drive_2.setPower(speed);
        robot.right_drive.setPower(speed);
        robot.right_drive_2.setPower(speed);
        //
        while (robot.left_drive.isBusy() && robot.right_drive.isBusy() && robot.left_drive_2.isBusy() && robot.right_drive_2.isBusy()) {
            referenceEncoder = robot.right_drive_2.getCurrentPosition();
            referenceEncoderDifference = referenceEncoder - lastReferenceEncoder;
            if (referenceEncoderDifference > 0) {
                referenceSpeed = referenceEncoderDifference / referenceCycleTime.seconds();
                referenceCycleTime.reset();
                lastReferenceEncoder = referenceEncoder;
                for (int i = 0; i < robot.drive_Motors.length - 1; i++) {
                    finalPid[i] = pid.getOutput(Math.abs(((robot.drive_Motors[i].getCurrentPosition()
                            - lastEncoders[i]) / motorCycleTimes[i].seconds()))
                            / robot.drive_Motors[i].getMotorType().getAchieveableMaxTicksPerSecond(), referenceEncoder);
                    motorCycleTimes[i].reset();
                }
            }
            MoveUtils.setEachMotor(robot.drive_Motors, new double[]{speed * finalPid[0], speed * finalPid[1], speed * finalPid[2], speed});
        }
        robot.right_drive.setPower(0);
        robot.left_drive.setPower(0);
        robot.right_drive_2.setPower(0);
        robot.left_drive_2.setPower(0);
        return;
    }

    public void tunedSyncStrafe(double inches, double speed, double P, double I, double D) {
        if (inches > 0) {
            currentDirection = Direction.RIGHT;
        } else currentDirection = Direction.LEFT;
        //
        int move = (int) (Math.round(inches * cpi * meccyBias));
        //

        double[] integral = {0, 0, 0};
        double[] derivative = {0, 0, 0};
        double[] lastError = {0, 0, 0};
        double[] finalPid = {0, 0, 0};

        double referenceSpeed;
        double lastReferenceEncoder = robot.right_drive_2.getCurrentPosition();
        double referenceEncoder = robot.right_drive_2.getCurrentPosition();
        double referenceEncoderDifference;

        PID pid = new PID(.0, .0, .0, 1);
        pid.setOutputFilter(.1);

        ElapsedTime referenceCycleTime = new ElapsedTime();
        ElapsedTime left1CycleTime = new ElapsedTime();
        ElapsedTime right1CycleTime = new ElapsedTime();
        ElapsedTime left2CycleTime = new ElapsedTime();
        ElapsedTime[] motorCycleTimes = {left1CycleTime, right1CycleTime, left2CycleTime};
        double[] lastEncoders = {robot.left_drive.getCurrentPosition(), robot.right_drive.getCurrentPosition(),
                robot.left_drive_2.getCurrentPosition()};

        robot.left_drive_2.setTargetPosition(robot.left_drive_2.getCurrentPosition() - move);
        robot.left_drive.setTargetPosition(robot.left_drive.getCurrentPosition() + move);
        robot.right_drive_2.setTargetPosition(robot.right_drive_2.getCurrentPosition() + move);
        robot.right_drive.setTargetPosition(robot.right_drive.getCurrentPosition() - move);
        //
        robot.left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        robot.left_drive.setPower(speed);
        robot.left_drive_2.setPower(speed);
        robot.right_drive.setPower(speed);
        robot.right_drive_2.setPower(speed);
        //
        while (robot.left_drive.isBusy() && robot.right_drive.isBusy() && robot.left_drive_2.isBusy() && robot.right_drive_2.isBusy()) {
            try {
                sleep(500);
            } catch (InterruptedException e) {
            }
            referenceEncoder = robot.right_drive_2.getCurrentPosition();
            referenceEncoderDifference = referenceEncoder - lastReferenceEncoder;
            if (referenceEncoderDifference > 0) {
                referenceSpeed = (referenceEncoderDifference / referenceCycleTime.seconds())
                        / robot.right_drive_2.getMotorType().getAchieveableMaxTicksPerSecond();
                pid.setSetpoint(referenceSpeed);
                referenceCycleTime.reset();
                lastReferenceEncoder = referenceEncoder;
                for (int i = 0; i < robot.drive_Motors.length - 1; i++) {
                    finalPid[i] = pid.getOutput(Math.abs(((robot.drive_Motors[i].getCurrentPosition()
                            - lastEncoders[i]) / motorCycleTimes[i].seconds()))
                            / robot.drive_Motors[i].getMotorType().getAchieveableMaxTicksPerSecond(), referenceEncoder);
                    motorCycleTimes[i].reset();
                }
            }
            MoveUtils.setEachMotor(robot.drive_Motors, new double[]{speed + finalPid[0], speed + finalPid[1], speed + finalPid[2], speed});
        }
        robot.right_drive.setPower(0);
        robot.left_drive.setPower(0);
        robot.right_drive_2.setPower(0);
        robot.left_drive_2.setPower(0);
        return;
    }

    public void syncForward(double inches, double speed) {
        if (inches > 0) {
            currentDirection = Direction.FORWARDS;
        } else currentDirection = Direction.BACKWARDS;
        //
        int move = (int) (Math.round(inches * cpi * meccyBias));
        //

        double[] integral = {0, 0, 0};
        double[] derivative = {0, 0, 0};
        double[] lastError = {0, 0, 0};
        double[] finalPid = {0, 0, 0};

        double referenceSpeed;
        double lastReferenceEncoder = robot.right_drive_2.getCurrentPosition();
        double referenceEncoder = robot.right_drive_2.getCurrentPosition();
        double referenceEncoderDifference;

        PID pid = new PID(1.4, 1.4, .8);
        pid.setOutputFilter(.1);

        ElapsedTime referenceCycleTime = new ElapsedTime();
        ElapsedTime left1CycleTime = new ElapsedTime();
        ElapsedTime right1CycleTime = new ElapsedTime();
        ElapsedTime left2CycleTime = new ElapsedTime();
        ElapsedTime[] motorCycleTimes = {left1CycleTime, right1CycleTime, left2CycleTime};
        double[] lastEncoders = {robot.left_drive.getCurrentPosition(), robot.right_drive.getCurrentPosition(),
                robot.left_drive_2.getCurrentPosition()};

        robot.left_drive_2.setTargetPosition(robot.left_drive_2.getCurrentPosition() + move);
        robot.left_drive.setTargetPosition(robot.left_drive.getCurrentPosition() + move);
        robot.right_drive_2.setTargetPosition(robot.right_drive_2.getCurrentPosition() + move);
        robot.right_drive.setTargetPosition(robot.right_drive.getCurrentPosition() + move);
        //
        robot.left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        robot.left_drive.setPower(speed);
        robot.left_drive_2.setPower(speed);
        robot.right_drive.setPower(speed);
        robot.right_drive_2.setPower(speed);
        //
        while (robot.left_drive.isBusy() && robot.right_drive.isBusy() && robot.left_drive_2.isBusy() && robot.right_drive_2.isBusy()) {
            referenceEncoder = robot.right_drive_2.getCurrentPosition();
            referenceEncoderDifference = referenceEncoder - lastReferenceEncoder;
            if (referenceEncoderDifference > 0) {
                referenceSpeed = referenceEncoderDifference / referenceCycleTime.seconds();
                referenceCycleTime.reset();
                lastReferenceEncoder = referenceEncoder;
                for (int i = 0; i < robot.drive_Motors.length - 1; i++) {
                    finalPid[i] = pid.getOutput(Math.abs(((robot.drive_Motors[i].getCurrentPosition()
                            - lastEncoders[i]) / motorCycleTimes[i].seconds()))
                            / robot.drive_Motors[i].getMotorType().getAchieveableMaxTicksPerSecond(), referenceEncoder);
                    motorCycleTimes[i].reset();
                }
            }
            MoveUtils.setEachMotor(robot.drive_Motors, new double[]{speed * finalPid[0], speed * finalPid[1], speed * finalPid[2], speed});
        }
        robot.right_drive.setPower(0);
        robot.left_drive.setPower(0);
        robot.right_drive_2.setPower(0);
        robot.left_drive_2.setPower(0);
        return;


    }

    public void startEncoderChecker() {
        encoderCheck = new Thread(() -> {
            try {
                while (!encoderCheck.isInterrupted()) {
                    for (int i = 0; i < encoders.length; i++) {
                        encoders[i] = robot.drive_Motors[i].getCurrentPosition();
                    }
                }
            } catch (Exception e) {
                throw new RuntimeException("Encoder Thread Interrupted");
            }
        });
        encoderCheck.start();
    }

    Integer linearCPR = 28; //counts per rotation
    Integer LinearGearRatio = 20; //NeverRest 20
    Double linearDiameter = 2.0;
    Double LinearCPI = (linearCPR * gearratio) / (linearDiameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))

    public void Linear(double Linear_Position, double timeoutSeconds, double inches) {

        int move = (int) (Math.round(inches * LinearCPI / 2));

        //robot.arm2.setTargetPosition(robot.arm2.getCurrentPosition() + move);
        robot.arm.setTargetPosition(robot.arm.getCurrentPosition() - move);


        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.arm.setPower(-Linear_Position);
        robot.arm2.setPower(Linear_Position);

        while (MoveUtils.areAllMotorsBusy(new DcMotor[]{robot.arm})) {
            robot.arm.setPower(-Linear_Position);
            robot.arm2.setPower(Linear_Position);
        }
        robot.arm.setPower(0);
        robot.arm2.setPower(0);
    }
}


