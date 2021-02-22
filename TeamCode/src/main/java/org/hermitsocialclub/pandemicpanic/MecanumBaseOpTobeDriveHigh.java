package org.hermitsocialclub.pandemicpanic;

import com.qualcomm.hardware.motors.GoBILDA5201Series;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.UltimateGoalConfiguration;
import org.hermitsocialclub.telecat.PersistantTelemetry;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TobeDriveHigh", group = "Hermit")

public class MecanumBaseOpTobeDriveHigh extends LinearOpMode {

    private PersistantTelemetry pt = new PersistantTelemetry(telemetry);
    UltimateGoalConfiguration robot = new UltimateGoalConfiguration();
    ElapsedTime runtime = new ElapsedTime();
    private boolean lastAMash = false;
    private boolean lastBMash = false;
    public boolean precisionMode = false;
    public double precisionModifier = 1.25;
    public double invertedControls = 1;
    //private double initialLeftTicks, initialRightTicks, initialTopTicks;
    private double ticksPerRevolution = MotorConfigurationType.getMotorType(GoBILDA5201Series.class).getTicksPerRev();
    private double tobeMaxEncoder = MotorConfigurationType.getMotorType(GoBILDA5201Series.class).getAchieveableMaxTicksPerSecond();;
    private double tobeSpeedThreeEncoder = tobeMaxEncoder * .5;
    private double tobeSpeedTwoEncoder = 0.6 * tobeMaxEncoder ;
    private double tobeSpeedOneEncoder = 0.25 * tobeMaxEncoder;
    private double tobeDistanceRatio = 0.0625; //based off observed distance of ring at max rpm
    private double tobePowerRatio;

    private final double WOBBLE_GRAB_INCREMENT = .02;



    //private DistanceSensor sonicHedgehogSensor;
    private DcMotorEx tobeFlywheel;
    private DcMotorEx outtake;

    private CRServo kicker;



    @Override
    public void runOpMode() throws InterruptedException {

        tobeFlywheel = hardwareMap.get(DcMotorEx.class, "tobeFlywheel");
        kicker = hardwareMap.get(CRServo.class,"kicker");
        outtake = hardwareMap.get(DcMotorEx.class,"takeruFlyOut");
        robot.init(hardwareMap);
        //sonicHedgehogSensor = hardwareMap.get(DistanceSensor.class,"Sonic the Hedgehog");
        //tobePowerRatio = Math.max(sonicHedgehogSensor.getDistance(DistanceUnit.CM) * tobeDistanceRatio,1);


        waitForStart();
        //initialLeftTicks = robot.leftEncoder.getCurrentPosition();
        //initialRightTicks = robot.rightEncoder.getCurrentPosition();
        //initialTopTicks = robot.frontEncoder.getCurrentPosition();
        telemetry.speak("Hola. Cómo estás?", "spa", "mx");
        telemetry.update();

        while (opModeIsActive()) {
            //tobePowerRatio = Math.max(sonicHedgehogSensor.getDistance(DistanceUnit.CM) * tobeDistanceRatio,1);

            if (!lastAMash && gamepad1.a) {

                if (precisionMode) {
                    precisionMode = false;
                    precisionModifier = 1.2;
                    pt.setData("Precision Mode", "DEACTIVATED!");

                } else {
                    precisionMode = true;
                    precisionModifier = 0.5;
                    pt.setData("Precision Mode", "ACTIVATED!");

                }
                runtime.reset();

            }
            lastAMash = gamepad1.a;

            if (!lastBMash && gamepad1.b) {
                if (invertedControls == 1) {
                    invertedControls = -1;
                    pt.setData("Inverse Controls", "ACTIVATED!");
                } else if (invertedControls == -1) {
                    invertedControls = 1;
                    pt.setData("Inverse Controls", "DEACTIVATED");
                }
            }
            lastBMash = gamepad1.b;

            double r = MoveUtils.joystickXYToRadius(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = MoveUtils.joystickXYToAngle(gamepad1.left_stick_x, gamepad1.left_stick_y);

            double[] powers = MoveUtils.theAlgorithm(r, robotAngle, -gamepad1.right_stick_x, precisionModifier * invertedControls);
            MoveUtils.setEachMotor(new DcMotor[]{robot.left_drive, robot.right_drive, robot.left_drive_2, robot.right_drive_2}, powers);
            if(gamepad1.left_bumper){
                tobeFlywheel.setPower(0);
            }
            else if (gamepad1.x) {
                outtake.setPower(0);
            }
            else if (gamepad1.dpad_up){
                tobeFlywheel.setPower(1);
                //tobeFlywheel.setVelocity(2 * Math.PI * -tobeMaxEncoder / ticksPerRevolution, AngleUnit.RADIANS);
            }else if (gamepad1.dpad_down){
                outtake.setPower(-0.9);
                //tobeFlywheel.setVelocity(2 * Math.PI * -tobeSpeedOneEncoder / ticksPerRevolution, AngleUnit.RADIANS);
            }else if (gamepad1.dpad_left){
                outtake.setPower(-1);
               // tobeFlywheel.setPower(.62);
                //tobeFlywheel.setVelocity(2 * Math.PI * -tobeSpeedTwoEncoder / ticksPerRevolution, AngleUnit.RADIANS);
            }else if (gamepad1.dpad_right){
                tobeFlywheel.setPower(.75);
                //tobeFlywheel.setVelocity(2 * Math.PI * -tobeSpeedThreeEncoder / ticksPerRevolution, AngleUnit.RADIANS);
            }
            //if (gamepad2.x){
                //tobeFlywheel.setPower(-tobePowerRatio);
           // }
            if(gamepad1.right_trigger > 0.5){
                kicker.setPower(1);
            }else if(gamepad1.left_trigger > 0.5) {
                kicker.setPower(-1);
            }else{
                kicker.setPower(0);
            }

            if(gamepad2.right_trigger > .05){
                robot.wobbleArm.setPower(gamepad2.right_trigger);
            }else if(gamepad2.left_trigger > .05){
                robot.wobbleArm.setPower( -gamepad2.left_trigger);
            } else robot.wobbleArm.setPower(0);
            if(gamepad2.right_bumper){
                robot.wobbleGrab.setPower(1);
            } else if(gamepad2.left_bumper){
                robot.wobbleGrab.setPower(-1);
            }else robot.wobbleGrab.setPower(0);

            //pt.setData("raw ultrasonic", sonicHedgehogSensor.getDistance(DistanceUnit.CM));
            //pt.setData("cm", "%.2f cm", sonicHedgehogSensor.getDistance(DistanceUnit.CM));
        }

    }



}
