/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.hermitsocialclub.pandemicpanic;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PushBotConfiguration {

    public DcMotor left_drive = null;
    public DcMotor left_drive_2 = null;
    public DcMotor right_drive = null;
    public DcMotor right_drive_2 = null;
    public DcMotor[] drive_Motors;
    public DcMotor duck_wheel = null;
    public DcMotor arm = null;
    public Servo claw = null;
   // public DcMotorEx leftEncoder, frontEncoder, rightEncoder;
    /*public DcMotor tapeShooter;
    public Servo foundation_Mover = null;
    public Servo foundation_Mover_2 = null;
    public BNO055IMU imu = null;
    public Servo block_Clamper = null;
    public DcMotor arm = null;
    public DcMotor arm2 = null;
    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;
    public Servo block_Clamper_2;
    public Servo capSlinger;
    public Servo topClaw;
    public DcMotorEx spinner;*/

  /*  public static final double MID_SERVO = 1;
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;
    public static final double SERVO_HOME = 0.0;
    public static final double SERVO_MIN = 0.0;
    public static final double SERVO_MAX = 0.65; */

    /* local OpMode members. */
    private HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /** Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        left_drive = hwMap.get(DcMotor.class, "left_drive");
        right_drive = hwMap.get(DcMotor.class, "right_drive");
        left_drive_2 = hwMap.get(DcMotor.class, "left_drive_2");
        right_drive_2 = hwMap.get(DcMotor.class, "right_drive_2");
        duck_wheel = hwMap.get(DcMotor.class, "duck_wheel");
        arm = hwMap.get(DcMotor.class,"arm");
        claw = hwMap.get(Servo.class,"claw");
     /*   foundation_Mover = hwMap.get(Servo.class, "Foundation_Mover");
        foundation_Mover_2 = hwMap.get(Servo.class, "Foundation_Mover_2");
        block_Clamper = hwMap.get(Servo.class, "Block_Clamper");
        arm = hwMap.get(DcMotor.class, "arm");
        arm2 = hwMap.get(DcMotor.class, "arm2");
        imu = hwMap.get(BNO055IMU.class, "imu");
        colorSensor = hwMap.get(ColorSensor.class, "Color Sensor");
        block_Clamper_2 = hwMap.get(Servo.class, "block clamper 2");
        tapeShooter = hwMap.get(DcMotor.class, "tapeShooter");
        capSlinger = hwMap.get(Servo.class, "capSlinger");
        topClaw = hwMap.get(Servo.class, "topClaw"); */
     /*   leftEncoder = hwMap.get(DcMotorEx.class, "leftEncoder");
        rightEncoder = hwMap.get(DcMotorEx.class, "tapeShooter");
        frontEncoder = hwMap.get(DcMotorEx.class, "arm"); */
      /*  spinner = hwMap.get(DcMotorEx.class,"tapeShooter" +
                "");
*/
        left_drive.setDirection(DcMotor.Direction.FORWARD);    // Set to REVERSE if using AndyMark motors
        right_drive.setDirection(DcMotor.Direction.REVERSE);   // Set to FORWARD if using AndyMark motors
        left_drive_2.setDirection(DcMotor.Direction.FORWARD);  // Set to REVERSE if using AndyMark motors
        right_drive_2.setDirection(DcMotor.Direction.REVERSE); // Set to FORWARD if using AndyMark motors
        duck_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);
      /*  arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm2.setDirection(DcMotorSimple.Direction.REVERSE); */

        // Set all motors to zero power
        left_drive.setPower(0);
        right_drive.setPower(0);
        left_drive_2.setPower(0);
        right_drive_2.setPower(0);
        duck_wheel.setPower(0);
        arm.setPower(0);
       // arm.setPower(0);
        //arm2.setPower(0);
        drive_Motors = new DcMotor[]{left_drive, right_drive, left_drive_2, right_drive_2};

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_drive_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_drive_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_drive_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duck_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize Rev Color sensor
     //   colorSensor.enableLed(true);

        // Initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

     //   imu.initialize(parameters);
      //  imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public HardwareMap getHardwareMap() {
        return hwMap;
    }

   /* public Trajectory trajectorybuilder(Pose2d pose2d) {

        return null;
    }*/
}

