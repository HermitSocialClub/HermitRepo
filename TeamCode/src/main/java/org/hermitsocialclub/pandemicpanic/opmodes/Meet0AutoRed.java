package org.hermitsocialclub.pandemicpanic.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Meet0Auto", group = "Hermit")
public class Meet0AutoRed extends LinearOpMode {

    DcMotor left_drive;
    DcMotor right_drive;
    DcMotor left_drive_2;
    DcMotor right_drive_2;
    DcMotor duck_wheel;
    DcMotor intake;
    DcMotor linear;
    //28 * 20 / (2ppi * 4.125)
    Double width = 18.0; //inches
    Integer cpr = 28; //counts per rotation
    Integer gear_ratio = 40;
    Double diameter = 4.125;
    Double cpi = (cpr * gear_ratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.8;//default 0.8
    Double meccyBias = 0.9;//change to adjust only strafing movement

    Double conversion = cpi * bias;
    Boolean exit = false;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

      //  @Autonomous(name="Meet0Auto", group="Hermit")
        //public class myAuto extends LinearOpMode {
            //

           // public void runOpMode(){
                //
                initGyro();
                //
                left_drive = hardwareMap.dcMotor.get("frontleft");
                right_drive = hardwareMap.dcMotor.get("frontright");
                left_drive_2 = hardwareMap.dcMotor.get("backleft");
                right_drive_2 = hardwareMap.dcMotor.get("backright");
                duck_wheel = hardwareMap.dcMotor.get("duckweel");
                intake = hardwareMap.dcMotor.get("intake");
                linear = hardwareMap.dcMotor.get("linear");

                right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
                right_drive_2.setDirection(DcMotorSimple.Direction.REVERSE);
                //
                waitForStart();
                //
                moveLinears(0.5,0.1);
                //
                moveToPosition(35.8, 0.5);
                //
/*
                moveLinears(0.5,0.1);
                moveToPosition(18.5,0.2);
                strafeToPosition(13,0.2);
                moveToPosition(-16,0.2);
                strafeToPosition(3,0.2);
                duckKnocker(3,0.2);
                moveToPosition(16,0.2);
                strafeToPosition(-50,0.3);
                moveLinears(1,0.1);
                strafeToPosition(-40,0.1);


                moveLinears(0.5,0.1);
                //
                strafeToPosition(18.5,0.2);
                //
                moveToPosition(-13,0.2);
                //
                strafeToPosition(-16,0.2);
                //
                moveToPosition(-3,0.2);
                //
                duckKnocker(3,0.2);
                //
                strafeToPosition(16,0.2);
                //
                moveToPosition(50,0.3);
                //
                moveLinears(1,0.1);
                //
                moveToPosition(40,0.5);
                //
                //intakerr(3,0.55);
                //alternative path to do carousel first and then back p into the warehouse
        */
            }
            //
    /*
    This function's purpose is simply to drive forward or backward.
    To drive backward, simply make the inches input negative.
     */
            public void moveToPosition(double inches, double speed){
                //
                int move = (int)(Math.round(inches*conversion));
                //
                left_drive_2.setTargetPosition(left_drive_2.getCurrentPosition() + move);
                left_drive.setTargetPosition(left_drive.getCurrentPosition() + move);
                right_drive_2.setTargetPosition(right_drive_2.getCurrentPosition() + move);
                right_drive.setTargetPosition(right_drive.getCurrentPosition() + move);
                //
                left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                left_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //
                left_drive.setPower(speed);
                left_drive_2.setPower(speed);
                right_drive.setPower(speed);
                right_drive_2.setPower(speed);
                //
                while (left_drive.isBusy() && right_drive.isBusy() && left_drive_2.isBusy() && right_drive_2.isBusy()){
                    if (exit){
                        right_drive.setPower(0);
                        left_drive.setPower(0);
                        right_drive_2.setPower(0);
                        left_drive_2.setPower(0);
                        return;
                    }
                }
                right_drive.setPower(0);
                left_drive.setPower(0);
                right_drive_2.setPower(0);
                left_drive_2.setPower(0);
                return;
            }
            //
    /*
    This function uses the Expansion Hub IMU Integrated Gyro to turn a precise number of degrees (+/- 5).
    Degrees should always be positive, make speedDirection negative to turn left.
     */
            public void turnWithGyro(double degrees, double speedDirection){
                //<editor-fold desc="Initialize">
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                double yaw = -angles.firstAngle;//make this negative
                telemetry.addData("Speed Direction", speedDirection);
                telemetry.addData("Yaw", yaw);
                telemetry.update();
                //
                telemetry.addData("stuff", speedDirection);
                telemetry.update();
                //
                double first;
                double second;
                //</editor-fold>
                //
                if (speedDirection > 0){//set target positions
                    //<editor-fold desc="turn right">
                    if (degrees > 10){
                        first = (degrees - 10) + addToDegrees(yaw);
                        second = degrees + addToDegrees(yaw);
                    }else{
                        first = addToDegrees(yaw);
                        second = degrees + addToDegrees(yaw);
                    }
                    //</editor-fold>
                }else{
                    //<editor-fold desc="turn left">
                    if (degrees > 10){
                        first = addToDegrees(-(degrees - 10) + addToDegrees(yaw));
                        second = addToDegrees(-degrees + addToDegrees(yaw));
                    }else{
                        first = addToDegrees(yaw);
                        second = addToDegrees(-degrees + addToDegrees(yaw));
                    }
                    //
                    //</editor-fold>
                }
                //
                //<editor-fold desc="Go to position">
                Double firsta = convertDegrees(first - 5);//175
                Double firstb = convertDegrees(first + 5);//-175
                //
                turnWithEncoder(speedDirection);
                //
                if (Math.abs(firsta - firstb) < 11) {
                    while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        gravity = imu.getGravity();
                        yaw = -angles.firstAngle;
                        telemetry.addData("Position", yaw);
                        telemetry.addData("first before", first);
                        telemetry.addData("first after", convertDegrees(first));
                        telemetry.update();
                    }
                }else{
                    //
                    while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        gravity = imu.getGravity();
                        yaw = -angles.firstAngle;
                        telemetry.addData("Position", yaw);
                        telemetry.addData("first before", first);
                        telemetry.addData("first after", convertDegrees(first));
                        telemetry.update();
                    }
                }
                //
                Double seconda = convertDegrees(second - 5);//175
                Double secondb = convertDegrees(second + 5);//-175
                //
                turnWithEncoder(speedDirection / 3);
                //
                if (Math.abs(seconda - secondb) < 11) {
                    while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        gravity = imu.getGravity();
                        yaw = -angles.firstAngle;
                        telemetry.addData("Position", yaw);
                        telemetry.addData("second before", second);
                        telemetry.addData("second after", convertDegrees(second));
                        telemetry.update();
                    }
                    while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        gravity = imu.getGravity();
                        yaw = -angles.firstAngle;
                        telemetry.addData("Position", yaw);
                        telemetry.addData("second before", second);
                        telemetry.addData("second after", convertDegrees(second));
                        telemetry.update();
                    }
                    left_drive.setPower(0);
                    right_drive.setPower(0);
                    left_drive_2.setPower(0);
                    right_drive_2.setPower(0);
                }
                //</editor-fold>
                //
                left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                left_drive_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right_drive_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                left_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            //
    /*
    This function uses the encoders to strafe left or right.
    Negative input for inches results in left strafing.
     */
            public void strafeToPosition(double inches, double speed){
                //
                int move = (int)(Math.round(inches * cpi * meccyBias));
                //
                left_drive_2.setTargetPosition(left_drive_2.getCurrentPosition() - move);
                left_drive.setTargetPosition(left_drive.getCurrentPosition() + move);
                right_drive_2.setTargetPosition(right_drive_2.getCurrentPosition() + move);
                right_drive.setTargetPosition(right_drive.getCurrentPosition() - move);
                //
                left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                left_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //
                left_drive.setPower(speed);
                left_drive_2.setPower(speed);
                right_drive.setPower(speed);
                right_drive_2.setPower(speed);
                //
                while (left_drive.isBusy() && right_drive.isBusy() && left_drive_2.isBusy() && right_drive_2.isBusy()){}
                right_drive.setPower(0);
                left_drive.setPower(0);
                right_drive_2.setPower(0);
                left_drive_2.setPower(0);
                return;
            }
            //
             public void duckKnocker(long timeout, double speed){
               /* duckwheel.setPower(speed);
                while (opModeIsActive() &&
                         (runtime.seconds() < timeout) &&
                         (frontleft.isBusy() && frontright.isBusy())&&(backleft.isBusy() && backright.isBusy())); */

                 long start = System.currentTimeMillis();
                 long end = start + timeout *1000;
                 while (System.currentTimeMillis() < end) {
                     duck_wheel.setPower(speed);
                 }
             }
    public void moveLinears (double timeout, double speed){

        double start = System.currentTimeMillis();
        double end = start + timeout *1000;
        while (System.currentTimeMillis() < end) {
            linear.setPower(speed);
        }
    }
    public void intakerr (long timeout, double speed){

        long start = System.currentTimeMillis();
        long end = start + timeout *1000;
        while (System.currentTimeMillis() < end) {
            duck_wheel.setPower(speed);
        }
    }
    /*
    A tradition within the Thunder Pengwins code, we always start programs with waitForStartify,
    our way of adding personality to our programs.
     */
          /*  public void waitForStartify(){
                waitForStart();
            } */
            //
    /*
    These functions are used in the turnWithGyro function to ensure inputs
    are interpreted properly.
     */
            public double addToDegrees(double degrees){
                if (degrees < 0){
                    degrees = degrees + 360;
                }
                return degrees;
            }
            public double convertDegrees(double degrees){
                if (degrees > 179){
                    degrees = -(360 - degrees);
                } else if(degrees < -180){
                    degrees = 360 + degrees;
                } else if(degrees > 360){
                    degrees = degrees - 360;
                }
                return degrees;
            }
            //
    /*
    This function is called at the beginning of the program to activate
    the IMU Integrated Gyro.
     */
            public void initGyro(){
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
                parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
                parameters.loggingEnabled      = true;
                parameters.loggingTag          = "IMU";
                parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
                //
                imu = hardwareMap.get(BNO055IMU.class, "imu");
                imu.initialize(parameters);
            }
            //
    /*
    This function is used in the turnWithGyro function to set the
    encoder mode and turn.
     */
            public void turnWithEncoder(double input){
                left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                left_drive_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                right_drive_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //
                left_drive.setPower(input);
                left_drive_2.setPower(input);
                right_drive.setPower(-input);
                right_drive_2.setPower(-input);
            }
            //
        }

