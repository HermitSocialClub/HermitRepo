package org.hermitsocialclub.opmodes.freightfrenzy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "BasicBigBotAutoBlue", group = "Hermit")
    public class BasicBigBotAutoBlue extends LinearOpMode {

 DcMotor left_drive;
 DcMotor right_drive;
 DcMotor left_drive_2;
 DcMotor right_drive_2;
 DcMotor duck_wheel;
 //DcMotor intake;
 //DcMotor linear;
 //28 * 20 / (2ppi * 4.125)
 Double width = 13.0; //inches
 Integer cpr = 28; //counts per rotation
 Integer gear_ratio = 40;
 Double diameter = 4.125;
 Double cpi = (cpr * gear_ratio) / (Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
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

  //initGyro();
  //
  left_drive = hardwareMap.dcMotor.get("left_drive");
  right_drive = hardwareMap.dcMotor.get("right_drive");
  left_drive_2 = hardwareMap.dcMotor.get("left_drive_2");
  right_drive_2 = hardwareMap.dcMotor.get("right_drive_2");
  duck_wheel = hardwareMap.dcMotor.get("duck_wheel");
  //intake = hardwareMap.dcMotor.get("intake");
  //linear = hardwareMap.dcMotor.get("linear");

  right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
  right_drive_2.setDirection(DcMotorSimple.Direction.REVERSE);
  //
  waitForStart();
  //
  strafeToPosition(10.0, 0.3);
  //
  moveToPosition(18, 0.3);
  //
  strafeToPosition(-97.4, 0.3);
  //
     }
 public void moveToPosition(double inches, double speed) {
  //
  int move = (int) (Math.round(inches * conversion));
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
  while (left_drive.isBusy() && right_drive.isBusy() && left_drive_2.isBusy() && right_drive_2.isBusy()) {
   if (exit) {
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

 public void strafeToPosition(double inches, double speed) {
  //
  int move = (int) (Math.round(inches * cpi * meccyBias));
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
  while (left_drive.isBusy() && right_drive.isBusy() && left_drive_2.isBusy() && right_drive_2.isBusy()) {
  }
  right_drive.setPower(0);
  left_drive.setPower(0);
  right_drive_2.setPower(0);
  left_drive_2.setPower(0);
  return;
 }

 public void duckKnocker(long timeout, double speed) {
               /* duckwheel.setPower(speed);
                while (opModeIsActive() &&
                         (runtime.seconds() < timeout) &&
                         (frontleft.isBusy() && frontright.isBusy())&&(backleft.isBusy() && backright.isBusy())); */

  long start = System.currentTimeMillis();
  long end = start + timeout * 1000;
  while (System.currentTimeMillis() < end) {
   duck_wheel.setPower(speed);
  }
 }
}
