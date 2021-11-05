package org.hermitsocialclub.pandemicpanic.opmodes;

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

public class Meet0Auto extends LinearOpMode {
   // @Override
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    DcMotor duckwheel;
    //28 * 20 / (2ppi * 4.125)
    Double width = 18.0; //inches
    Integer cpr = 28; //counts per rotation
    Integer gearratio = 40;
    Double diameter = 4.125;
    Double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.8;//default 0.8
    Double meccyBias = 0.9;//change to adjust only strafing movement
    //
    Double conversion = cpi * bias;
    Boolean exit = false;
    //
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    //
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
                frontleft = hardwareMap.dcMotor.get("frontleft");
                frontright = hardwareMap.dcMotor.get("frontright");
                backleft = hardwareMap.dcMotor.get("backleft");
                backright = hardwareMap.dcMotor.get("backright");
                duckwheel = hardwareMap.dcMotor.get("duckweel");

                frontright.setDirection(DcMotorSimple.Direction.REVERSE);
                backright.setDirection(DcMotorSimple.Direction.REVERSE);
                //
                waitForStart();
                //
                moveToPosition(35.8, 0.5);

                //
               /*
               moveToPosition(16,02);
               //
                duckKnocker(3,0.2);
               //
                moveToPosition(-95,0.5);*/
                //alternative path to do carousel first and then back p into the warehouse
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
                backleft.setTargetPosition(backleft.getCurrentPosition() + move);
                frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
                backright.setTargetPosition(backright.getCurrentPosition() + move);
                frontright.setTargetPosition(frontright.getCurrentPosition() + move);
                //
                frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //
                frontleft.setPower(speed);
                backleft.setPower(speed);
                frontright.setPower(speed);
                backright.setPower(speed);
                //
                while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){
                    if (exit){
                        frontright.setPower(0);
                        frontleft.setPower(0);
                        backright.setPower(0);
                        backleft.setPower(0);
                        return;
                    }
                }
                frontright.setPower(0);
                frontleft.setPower(0);
                backright.setPower(0);
                backleft.setPower(0);
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
                    frontleft.setPower(0);
                    frontright.setPower(0);
                    backleft.setPower(0);
                    backright.setPower(0);
                }
                //</editor-fold>
                //
                frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
                backleft.setTargetPosition(backleft.getCurrentPosition() - move);
                frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
                backright.setTargetPosition(backright.getCurrentPosition() + move);
                frontright.setTargetPosition(frontright.getCurrentPosition() - move);
                //
                frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //
                frontleft.setPower(speed);
                backleft.setPower(speed);
                frontright.setPower(speed);
                backright.setPower(speed);
                //
                while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){}
                frontright.setPower(0);
                frontleft.setPower(0);
                backright.setPower(0);
                backleft.setPower(0);
                return;
            }
            //
             public void duckKnocker(double timeout, double speed){
                duckwheel.setPower(speed);
                while (opModeIsActive() &&
                         (runtime.seconds() < timeout) &&
                         (frontleft.isBusy() && frontright.isBusy())&&(backleft.isBusy() && backright.isBusy()));
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
                frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //
                frontleft.setPower(input);
                backleft.setPower(input);
                frontright.setPower(-input);
                backright.setPower(-input);
            }
            //
        }

