package org.hermitsocialclub.opmodes.pandemicpanic;


import static org.hermitsocialclub.drive.config.DriveConstants.HEADING_PID;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.motors.GoBILDA5201Series;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.opmodes.freightfrenzy.PoseStorage;
import org.hermitsocialclub.telecat.PersistantTelemetry;
import org.hermitsocialclub.util.MoveUtils;
import org.hermitsocialclub.util.UltimateGoalConfiguration;

@Disabled
@TeleOp(name = "Version 2 2021 Mecanum Base Op", group = "Hermit")
public class Ver2MecanumBaseOp2021 extends LinearOpMode {

    private final PersistantTelemetry pt = new PersistantTelemetry(telemetry);
    private final UltimateGoalConfiguration robot = new UltimateGoalConfiguration();
    public static double DRAWING_TARGET_RADIUS = 2;
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime kickTime = new ElapsedTime();
    private static final double WOBBLE_GRAB_INCREMENT = 0.02;
    private static double HOPPER_POSITION = 0;

    private boolean lastAMash = false;
    private boolean lastBMash = false;
    private boolean lastDownMash = false;
    private boolean lastUpMash = false;
    private boolean lastLeftMash = false;
    public boolean precisionMode = false;
    public double precisionModifier = 1.25;
    public double invertedControls = 1;
    public static double SPEED_PERCENT = 0.675;
    private double powerShotSpeed;
    public static double POWER_PERCENT = 0.595;
    private final double ticksPerRevolution = MotorConfigurationType.getMotorType(GoBILDA5201Series.class).getTicksPerRev();
    private final double tobeMaxEncoder = MotorConfigurationType.getMotorType(GoBILDA5201Series.class).getAchieveableMaxTicksPerSecond();
    private Vector2d powerLaunchVector = new Vector2d(-14,12.50);

    private final double tobeSpeedThreeEncoder = tobeMaxEncoder * 0.5;
    private final double tobeSpeedTwoEncoder = 0.6 * tobeMaxEncoder;
    private final double tobeSpeedOneEncoder = 0.25 * tobeMaxEncoder;
    private final double tobeDistanceRatio = 0.0625; // based off observed distance of ring at max rpm
    private double tobePowerRatio;

    private boolean kickFinished = true;
    private final long kickInterval = 200;
    private boolean kickDirection = false;
    private int kicks = 0;

    private final MotorConfigurationType goBildaOuttake = MotorConfigurationType.getMotorType(GoBILDA5202Series.class);
    private BaselineMecanumDrive drive;
    private boolean hopperMash = false;


    enum Mode {
        NORMAL_CONTROL, ALIGN_TO_POINT
    }

    private Mode mode = Mode.NORMAL_CONTROL;
    private PIDFController headingController = new PIDFController(HEADING_PID);

private boolean wobbleGrabLock = false;
private  boolean lastXMash = false;
private boolean kickStarting = false;
private boolean lastYMash = false;
private boolean alwaysOn = false;

    //private DistanceSensor sonicHedgehogSensor;
    private DcMotorEx intake;
    private DcMotorEx outtake;
    private Servo kicker;

    @Override
    public void runOpMode() throws InterruptedException {
        double maxKicks = 5;
        this.intake = hardwareMap.get(DcMotorEx.class, "tobeFlywheel");
        this.kicker = hardwareMap.get(Servo.class, "kicker");
        this.outtake = hardwareMap.get(DcMotorEx.class, "takeruFlyOut");
        this.outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.robot.init(hardwareMap);
        this.drive = new BaselineMecanumDrive(hardwareMap, pt);
        this.drive.setPoseEstimate(PoseStorage.currentPose);
        this.pt.setDebug("Pose", drive.getPoseEstimate());
        this.pt.setDebug("intendedPose", PoseStorage.currentPose);

        double outTake75Speed = -((SPEED_PERCENT * 2 * Math.PI * goBildaOuttake.getMaxRPM() * goBildaOuttake.getAchieveableMaxRPMFraction()) / 60);
        powerShotSpeed = -((POWER_PERCENT * 2 * Math.PI * goBildaOuttake.getMaxRPM() * goBildaOuttake.getAchieveableMaxRPMFraction()) / 60);

        headingController.setInputBounds(-Math.PI,Math.PI);
        kicker.setPosition(0.3);
        Trajectory constantLaunchSpline2 = drive.trajectoryBuilder(new Pose2d(powerLaunchVector,0))
                .splineToConstantHeading(new Vector2d(-3,-24.50),0)
                .addSpatialMarker(new Vector2d(0,-2.50), () -> launchRing(1,powerShotSpeed))
                .addSpatialMarker(new Vector2d(0,-14.50), () -> launchRing(1,powerShotSpeed))
                .addSpatialMarker(new Vector2d(0,-20.50), () -> launchRing(1,powerShotSpeed))
                .build();
        Trajectory[] trajectories = {
                drive.trajectoryBuilder(new Pose2d(48, 48, 0), 0)// 0
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(48, 24, 0), 0)//1
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(48, 0, 0), 0)//2
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(48, -24, 0), 0)//3
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(48, -48, 0), 0)//4
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(24, 48, 0), 0)//5
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(24, 24, 0), 0)//6
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(24, 0, 0), 0)//7
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(24, -24, 0), 0)//8
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(24, -48, 0), 0)//9
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(0, 48, 0), 0)//10
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(0, 24, 0), 0)//11
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(0, 0, 0), 0)//12
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(0, -24, 0), 0)//16
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(0, -48, 0), 0)//17
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(-24, 48, 0), 0)//18
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(-24, 24, 0), 0)//19
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(-24, 0, 0), 0)//20
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(-24, -24, 0), 0)//21
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(-24, -48, 0), 0)//22
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(-48, 48, 0), 0)//23
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(-48, 24, 0), 0)//24
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(-48, 0, 0), 0)//25
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(-48, -24, 0), 0)//26
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(-48, -48, 0), 0)//27
                        .lineToConstantHeading(new Vector2d(-3, -36))
                        .build(),
        };

        pt.setDebug("paths", "done");
        pt.setData("Inverse Controls", "DEACTIVATED");
        pt.setData("Precision Mode", "DEACTIVATED!");
        pt.setData("Wobble Grabber", "UNLOCKED");
        pt.setData("Outtake", "Turns Off");


        //sonicHedgehogSensor = hardwareMap.get(DistanceSensor.class,"Sonic the Hedgehog");
        //tobePowerRatio = Math.max(sonicHedgehogSensor.getDistance(DistanceUnit.CM) * tobeDistanceRatio,1);

        waitForStart();
        //initialLeftTicks = robot.leftEncoder.getCurrentPosition();
        //initialRightTicks = robot.rightEncoder.getCurrentPosition();
        //initialTopTicks = robot.frontEncoder.getCurrentPosition();
        pt.getOriginalTelemetry().speak("Hola. Cómo estás?", "spa", "mx");
        pt.getOriginalTelemetry().update();

        while (opModeIsActive()) {
            // Make sure to call drive.update() on *every* loop
            // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy
            drive.update();

            // Retrieve **our** pose
            Pose2d ourPose = drive.getPoseEstimate();
            if (gamepad1.right_stick_button) {
                double minDist = Math.sqrt(Math.pow((ourPose.getX() - trajectories[0].start().getX()), 2)
                        + Math.pow((ourPose.getY() - trajectories[0].start().getY()), 2));
                int closest = 0;
                for (int i = 1; i < trajectories.length; i++) {
                    double dist = Math.sqrt(Math.pow((ourPose.getX() - trajectories[i].start().getX()), 2)
                            + Math.pow((ourPose.getY() - trajectories[i].start().getY()), 2));
                    if (dist < minDist) {
                        minDist = dist;
                        closest = i;
                    }
                }
                drive.followTrajectory(trajectories[closest]);
            }
            pt.setDebug("x", ourPose.getX());
            pt.setDebug("y", ourPose.getY());
            pt.setDebug("heading", ourPose.getHeading());
            //tobePowerRatio = Math.max(sonicHedgehogSensor.getDistance(DistanceUnit.CM) * tobeDistanceRatio,1);

            if (!lastAMash && gamepad1.cross) {
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
            lastAMash = gamepad1.cross;

            if (!lastBMash && gamepad1.circle) {
                if (invertedControls == 1) {
                    invertedControls = -1;
                    pt.setData("Inverse Controls", "ACTIVATED!");
                } else if (invertedControls == -1) {
                    invertedControls = 1;
                    pt.setData("Inverse Controls", "DEACTIVATED");
                }
            }
            lastBMash = gamepad1.circle;

            if (!lastYMash && gamepad1.triangle) {
                if (alwaysOn) {
                    alwaysOn = false;
                    pt.setData("Outtake", "Turns Off");
                } else if (!alwaysOn) {
                    alwaysOn = true;
                    pt.setData("Outtake", "Always On");
                }
            }
            lastYMash = gamepad1.triangle;

            if (!lastXMash && gamepad1.square) {
                if (!wobbleGrabLock) {
                    wobbleGrabLock = true;
                    pt.setData("Wobble Grabber","LOCKED");
                }else {
                    wobbleGrabLock = false;
                    pt.setData("Wobble Grabber", "UNLOCKED");
                }
            }
            lastXMash = gamepad1.square;


            double r = MoveUtils.joystickXYToRadius(antiDeadzone(gamepad1.left_stick_x),
                    -antiDeadzone(gamepad1.left_stick_y) * invertedControls);
            double robotAngle = MoveUtils.joystickXYToAngle(antiDeadzone(gamepad1.left_stick_x) * invertedControls,
                    antiDeadzone(gamepad1.left_stick_y) * invertedControls);

            double[] powers = MoveUtils.theAlgorithm(r, robotAngle, antiDeadzone(gamepad1.right_stick_x), precisionModifier);
            MoveUtils.setEachMotor(new DcMotor[]{robot.left_drive, robot.right_drive, robot.left_drive_2, robot.right_drive_2}, powers);

            if (gamepad1.right_bumper) {
                intake.setPower(-0.8);
            } else if (gamepad1.left_bumper) {
                intake.setPower(0);
            }
            if(!hopperMash && gamepad1.left_stick_button){
                if(Math.abs(robot.hopperLift.getPosition() - .25) < .01) {
                    robot.hopperLift.setPosition(0);
                }else if(Math.abs(robot.hopperLift.getPosition()) <.01 ){
                    robot.hopperLift.setPosition(.25);
                }
            }
            hopperMash = gamepad1.left_stick_button;

            if (gamepad1.right_trigger > 0.02 && kickFinished) {
                kickFinished = false;
                kickTime.reset();
                if(runtime.seconds() > 90){outtake.setVelocity(-powerShotSpeed,AngleUnit.RADIANS);}
                else{
                outtake.setVelocity(-outTake75Speed, AngleUnit.RADIANS);}
            }

            if (gamepad1.right_trigger > 0.02 && !kickFinished && kickTime.milliseconds() >= 200) {
                if (runtime.seconds() > 90) {
                    maxKicks = 2;
                }else {
                    maxKicks = 5;
                }
                kickStarting = true;
            }

            if (kickStarting && !kickFinished &&
                    Math.abs(outtake.getVelocity(AngleUnit.RADIANS) + ((runtime.seconds() > 90) ? powerShotSpeed :outTake75Speed)) < Math.pow(10, -1)
                    /*&& drive.getPoseVelocity().getX() < 0.005  && drive.getPoseVelocity().getY() < 0.005
                    && drive.getPoseVelocity().getHeading() < 0.005*/) {

                if (kicks >= maxKicks && kickTime.milliseconds() >= kickInterval
                        && Math.abs(outtake.getVelocity(AngleUnit.RADIANS) + ((runtime.seconds() > 90) ? powerShotSpeed :outTake75Speed)) < Math.pow(10, -1)) {
                    kicker.setPosition(.1);
                    if(!alwaysOn  && runtime.seconds() > 90) {
                        outtake.setVelocity(0);
                    }
                    kicks = 0;
                    kickFinished = true;
                    kickStarting = false;
                }

                if (kickTime.milliseconds() >= kickInterval) {
                    if(kickDirection){
                        kickDirection = false;
                        kicker.setPosition(.9);
                    }else {
                        kickDirection = true;
                        kicker.setPosition(.1);
                    }
                    kickTime.reset();
                    if (!kickDirection) {
                        kicks++;
                    }
                }

                if (kicks == 0) {
                    kickTime.reset();
                }

            }
            if (gamepad1.left_trigger > 0.3) {
                outtake.setVelocity(0);
                kicker.setPosition(0.3);
                kickFinished = true;
                kickStarting = false;
            }

            if (!lastDownMash && gamepad2.dpad_down) {
                if (robot.intakeThirdStage.getPower() != 0) {
                    robot.intakeThirdStage.setPower(0);
                } else if (robot.intakeThirdStage.getPower() == 0) {
                    robot.intakeThirdStage.setPower(-1);
                }
            }
            lastDownMash = gamepad2.dpad_down;

            if (!lastUpMash && gamepad2.dpad_up) {
                if (robot.intakeThirdStage.getPower() != 0) {
                    robot.intakeThirdStage.setPower(0);
                } else if (robot.intakeThirdStage.getPower() == 0) {
                    robot.intakeThirdStage.setPower(1);
                }
            }
            lastUpMash = gamepad2.dpad_up;

            if (!lastLeftMash && gamepad1.dpad_left) {
                if (intake.getPower() != 0) {
                    intake.setPower(0);
                } else if (intake.getPower() == 0) {
                    intake.setPower(-.8);
                }
            }
            lastLeftMash = gamepad1.dpad_left;

            //if (gamepad1.square) {
            //   tobeFlywheel.setPower(-tobePowerRatio);
            //}

            if (gamepad2.right_trigger > 0.02) {
                HOPPER_POSITION += WOBBLE_GRAB_INCREMENT;
                robot.hopperLift.setPosition(HOPPER_POSITION);
            } else if (gamepad2.left_trigger > 0.02) {
                HOPPER_POSITION -= WOBBLE_GRAB_INCREMENT;
                robot.hopperLift.setPosition(HOPPER_POSITION);
            }
            pt.setDebug("Hopper Position",HOPPER_POSITION);
            if(Math.abs(gamepad2.left_stick_y) > .02){
                robot.wobbleArm.setPower(antiDeadzone(gamepad2.left_stick_y) * .35);
            }else if(Math.abs(gamepad2.left_stick_y) < -.02){
                robot.wobbleArm.setPower(0);
            }
            if (gamepad2.right_bumper) {
                robot.wobbleGrab.setPower(1);
            } else if (gamepad2.left_bumper) {
                robot.wobbleGrab.setPower(-1);
            }
           /* if(gamepad1.left_stick_button && runtime.seconds() >= 90){
                Trajectory constantLaunchSpline = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(powerLaunchVector,0),0)
                        .build();

                drive.followTrajectory(constantLaunchSpline);
                drive.outtake.setVelocity(powerShotSpeed);
                while (opModeIsActive() && Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) - powerShotSpeed) > Math.pow(10, -1)) {
                    pt.setDebug("Outtake Velocity", drive.outtake.getVelocity(AngleUnit.RADIANS));
                    pt.setDebug("Ticks Outtake Velocity", drive.outtake.getVelocity(AngleUnit.DEGREES));
                    pt.setDebug("Ticks Outtake Position", drive.outtake.getCurrentPosition());
                }
                drive.followTrajectory(constantLaunchSpline2);
            }*/

            //telemetry.setData("raw ultrasonic", sonicHedgehogSensor.getDistance(DistanceUnit.CM));
            //telemetry.setData("cm", "%.2f cm", sonicHedgehogSensor.getDistance(DistanceUnit.CM));
            pt.setDebug("Outtake Actual Speed", outtake.getVelocity(AngleUnit.RADIANS));
            pt.setDebug("Outtake Intended Speed", outTake75Speed);
            pt.setDebug("Kick Speed", kicker.getPosition());
            pt.setDebug("Kicks Completed", kicks);
            pt.setDebug("Kicking?", !kickFinished);
            pt.setDebug("kick interval", kickInterval);
            pt.setDebug("kickTime", kickTime);
            pt.setDebug("outtake error", Math.abs(outtake.getVelocity(AngleUnit.RADIANS) - outTake75Speed));
            pt.setDebug("error threshhold", Math.pow(10, -1) * 3);
            pt.setDebug("error difference", Math.pow(10, -1) - Math.abs(outtake.getVelocity(AngleUnit.RADIANS) - outTake75Speed));
        }

    }
    public double antiDeadzone (double input){
        return (Math.copySign(Math.max(Math.abs(input) * (1.0/.8) - .2,0),input));
    }
    private void launchRing(int ringsToFire, double speed){

        drive.outtake.setVelocity(speed, AngleUnit.RADIANS);
        while (opModeIsActive() && Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) - speed) > Math.pow(10, -1)) {
            pt.setDebug("Outtake Velocity", drive.outtake.getVelocity(AngleUnit.RADIANS));
            pt.setDebug("Ticks Outtake Velocity", drive.outtake.getVelocity(AngleUnit.DEGREES));
            pt.setDebug("Ticks Outtake Position", drive.outtake.getCurrentPosition());
        }
        int ringsFired = 0;
        while (opModeIsActive() && ringsFired < ringsToFire) {
            pt.setDebug("ringsFired", ringsFired);
            drive.kicker.setPosition(.7);
            sleep(200);
            drive.kicker.setPosition(.3);
            while (Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) - speed ) > Math.pow(10, -1)) {
                pt.setDebug("Outtake Velocity", drive.outtake.getVelocity(AngleUnit.RADIANS));
            }
            ringsFired++;
        }
        drive.outtake.setVelocity(0);

    }


}
