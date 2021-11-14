package org.hermitsocialclub.pandemicpanic;

import com.qualcomm.hardware.motors.GoBILDA5201Series;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.hermitsocialclub.telecat.PersistantTelemetry;
import org.hermitsocialclub.util.UltimateGoalConfiguration;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "2021 Mecanum Base Op", group = "Hermit")

public class MecanumBaseOp2021 extends LinearOpMode {

    private final PersistantTelemetry pt = new PersistantTelemetry(telemetry);
    UltimateGoalConfiguration robot = new UltimateGoalConfiguration();
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime kickTime = new ElapsedTime();
    private boolean lastAMash = false;
    private boolean lastBMash = false;
    public boolean precisionMode = false;
    public double precisionModifier = 1.25;
    public double invertedControls = 1;
    public static double SPEED_PERCENT = .615;
    //private double initialLeftTicks, initialRightTicks, initialTopTicks;
    private final double ticksPerRevolution = MotorConfigurationType.getMotorType(GoBILDA5201Series.class).getTicksPerRev();
    private final double tobeMaxEncoder = MotorConfigurationType.getMotorType(GoBILDA5201Series.class).getAchieveableMaxTicksPerSecond();
    private final double tobeSpeedThreeEncoder = tobeMaxEncoder * .5;
    private final double tobeSpeedTwoEncoder = 0.6 * tobeMaxEncoder;
    private final double tobeSpeedOneEncoder = 0.25 * tobeMaxEncoder;
    private final double tobeDistanceRatio = 0.0625; //based off observed distance of ring at max rpm
    private double tobePowerRatio;

    private boolean kickFinished = true;
    private final long kickInterval = 200;
    private int kickDirection = -1;
    private int kicks = 0;


    MotorConfigurationType goBildaOuttake = MotorConfigurationType.getMotorType(GoBILDA5202Series.class);
    private double outTake75Speed;

    private final double WOBBLE_GRAB_INCREMENT = .02;

    private boolean wobbleGrabLock = false;
    private final boolean lastXMash = false;

    //private DistanceSensor sonicHedgehogSensor;
    private DcMotorEx intake;
    private DcMotorEx outtake;

    private CRServo kicker;


    @Override
    public void runOpMode() throws InterruptedException {

        intake = hardwareMap.get(DcMotorEx.class, "tobeFlywheel");
        kicker = hardwareMap.get(CRServo.class, "kicker");
        outtake = hardwareMap.get(DcMotorEx.class, "takeruFlyOut");
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.init(hardwareMap);

        outTake75Speed = ((SPEED_PERCENT * 2 * Math.PI * goBildaOuttake.getMaxRPM() * goBildaOuttake.getAchieveableMaxRPMFraction()) / 60);


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
            if (!lastXMash && gamepad1.x) {
                if (!wobbleGrabLock) {
                    wobbleGrabLock = true;
                    pt.setData("Wobble Grabber", "LOCKED");
                } else {
                    wobbleGrabLock = false;
                    pt.setData("Wobble Grabber", "UNLOCKED");
                }
            }

            double r = MoveUtils.joystickXYToRadius(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = MoveUtils.joystickXYToAngle(-gamepad1.left_stick_x, gamepad1.left_stick_y);

            double[] powers = MoveUtils.theAlgorithm(r, robotAngle, gamepad1.right_stick_x, precisionModifier * invertedControls);
            MoveUtils.setEachMotor(new DcMotor[]{robot.left_drive, robot.right_drive, robot.left_drive_2, robot.right_drive_2}, powers);
           /* if(gamepad1.left_bumper){
                intake.setPower(0);
            }else if (gamepad1.right_bumper){
                intake.setPower(.8);
                //tobeFlywheel.setVelocity(2 * Math.PI * -tobeMaxEncoder / ticksPerRevolution, AngleUnit.RADIANS);
            }*/
            /*if (gamepad1.left_trigger > .2) {
                outtake.setPower(0);
            }
            else if (gamepad1.right_trigger > .2){
                outtake.setPower(-0.9);
                //tobeFlywheel.setVelocity(2 * Math.PI * -tobeSpeedOneEncoder / ticksPerRevolution, AngleUnit.RADIANS);
            }*/

            if (gamepad1.right_trigger > 0.2) {
                kickFinished = false;
                outtake.setVelocity(-outTake75Speed, AngleUnit.RADIANS);
            }
            /*if(gamepad1.right_bumper){
                kicker.setPower(1);
            }else if(gamepad1.left_bumper){
                kicker.setPower(-1);
            }else kicker.setPower(0);*/

            if (!kickFinished && Math.abs(-outtake.getVelocity(AngleUnit.RADIANS) - outTake75Speed) < Math.pow(10, -2) * 5) {
                if (kicks == 0) {
                    kickTime.reset();
                }
                if (kicks >= 5 && kickTime.milliseconds() >= kickInterval) {
                    kicker.setPower(0);
                    outtake.setVelocity(0);
                    kicks = 0;
                    kickFinished = true;
                }
                if (kickTime.milliseconds() >= kickInterval) {
                    kickDirection = kickDirection * -1;
                    kicker.setPower(kickDirection);
                    kickTime.reset();
                }
                if (kickDirection == -1) {
                    kicks++;
                }

            }
            if (gamepad1.left_trigger > .3) {
                outtake.setVelocity(0);
                kicker.setPower(0);
                kickFinished = true;
            }

            //if (gamepad2.x){
            //tobeFlywheel.setPower(-tobePowerRatio);
            // }

            if (gamepad2.right_trigger > .05) {
                robot.wobbleArm.setPower(gamepad2.right_trigger);
            } else if (gamepad2.left_trigger > .05) {
                robot.wobbleArm.setPower(-gamepad2.left_trigger);
            } else robot.wobbleArm.setPower(0);
            if (gamepad2.right_bumper) {
                robot.wobbleGrab.setPower(1);
            } else if (gamepad2.left_bumper) {
                robot.wobbleGrab.setPower(-1);
            } else if (!wobbleGrabLock) robot.wobbleGrab.setPower(0);

            //pt.setData("raw ultrasonic", sonicHedgehogSensor.getDistance(DistanceUnit.CM));
            //pt.setData("cm", "%.2f cm", sonicHedgehogSensor.getDistance(DistanceUnit.CM));
            pt.setDebug("Outtake Actual Speed", outtake.getVelocity(AngleUnit.RADIANS));
            pt.setDebug("Outtake Intended Speed", outTake75Speed);
            pt.setDebug("Kick Speed", kicker.getPower());
            pt.setDebug("Kicks Completed", kicks);
            pt.setDebug("Kicking?", !kickFinished);
            pt.setDebug("kick interval", kickInterval);
            pt.setDebug("kickTime", kickTime);
            pt.setDebug("outtake error", Math.abs(outtake.getVelocity(AngleUnit.RADIANS) - outTake75Speed));
            pt.setDebug("error threshhold", Math.pow(10, -1) * 3);
            pt.setDebug("error difference", Math.pow(10, -1) - Math.abs(outtake.getVelocity(AngleUnit.RADIANS) - outTake75Speed));
        }

    }

    public void kickerTime(long time, double power) {
        long beginning = System.currentTimeMillis();
        long end = beginning + time;
        while (end > System.currentTimeMillis()) {
            kicker.setPower(power);
        }
    }


}
