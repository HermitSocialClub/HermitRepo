package org.hermitsocialclub.pandemicpanic;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.motors.GoBILDA5201Series;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.opmode.BaselineMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;
import org.firstinspires.ftc.teamcode.util.UltimateGoalConfiguration;
import org.hermitsocialclub.telecat.PersistantTelemetry;

@TeleOp(name = "Version 2 2021 Mecanum Base Op", group = "Hermit")
public class Ver2MecanumBaseOp2021 extends LinearOpMode {

    private static final double WOBBLE_GRAB_INCREMENT = 0.02;

    private final PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);
    private final UltimateGoalConfiguration robot = new UltimateGoalConfiguration();
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime kickTime = new ElapsedTime();
    private boolean lastAMash = false;
    private boolean lastBMash = false;
    public boolean precisionMode = false;
    public double precisionModifier = 1.25;
    public double invertedControls = 1;
    public static double SPEED_PERCENT = 0.615;
    private final double ticksPerRevolution = MotorConfigurationType.getMotorType(GoBILDA5201Series.class).getTicksPerRev();
    private final double tobeMaxEncoder = MotorConfigurationType.getMotorType(GoBILDA5201Series.class).getAchieveableMaxTicksPerSecond();

    private final double tobeSpeedThreeEncoder = tobeMaxEncoder * 0.5;
    private final double tobeSpeedTwoEncoder = 0.6 * tobeMaxEncoder;
    private final double tobeSpeedOneEncoder = 0.25 * tobeMaxEncoder;
    private final double tobeDistanceRatio = 0.0625; // based off observed distance of ring at max rpm
    private double tobePowerRatio;

    private boolean kickFinished = true;
    private final long kickInterval = 300;
    private int kickDirection = -1;
    private int kicks = 0;

    private final MotorConfigurationType goBildaOuttake = MotorConfigurationType.getMotorType(GoBILDA5202Series.class);
    private BaselineMecanumDrive drive;

    private boolean wobbleGrabLock = false;
    private final boolean lastXMash = false;
    private boolean kickStarting = false;

    //private DistanceSensor sonicHedgehogSensor;
    private DcMotorEx intake;
    private DcMotorEx outtake;
    private CRServo kicker;

    @Override
    public void runOpMode() throws InterruptedException {
        this.intake = hardwareMap.get(DcMotorEx.class, "tobeFlywheel");
        this.kicker = hardwareMap.get(CRServo.class, "kicker");
        this.outtake = hardwareMap.get(DcMotorEx.class, "takeruFlyOut");
        this.outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.robot.init(hardwareMap);
        this.drive = new BaselineMecanumDrive(hardwareMap, telemetry);
        this.drive.setPoseEstimate(PoseStorage.currentPose);
        this.telemetry.setDebug("Pose", drive.getPoseEstimate());
        this.telemetry.setDebug("intendedPose", PoseStorage.currentPose);

        double outTake75Speed = -((SPEED_PERCENT * 2 * Math.PI * goBildaOuttake.getMaxRPM() * goBildaOuttake.getAchieveableMaxRPMFraction()) / 60);

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

        telemetry.setDebug("paths", "done");
        telemetry.setData("Inverse Controls", "DEACTIVATED");
        telemetry.setData("Precision Mode", "DEACTIVATED!");
        telemetry.setData("Wobble Grabber", "UNLOCKED");

        //sonicHedgehogSensor = hardwareMap.get(DistanceSensor.class,"Sonic the Hedgehog");
        //tobePowerRatio = Math.max(sonicHedgehogSensor.getDistance(DistanceUnit.CM) * tobeDistanceRatio,1);

        waitForStart();
        //initialLeftTicks = robot.leftEncoder.getCurrentPosition();
        //initialRightTicks = robot.rightEncoder.getCurrentPosition();
        //initialTopTicks = robot.frontEncoder.getCurrentPosition();
        telemetry.getOriginalTelemetry().speak("Hola. Cómo estás?", "spa", "mx");
        telemetry.getOriginalTelemetry().update();

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
            telemetry.setDebug("x", ourPose.getX());
            telemetry.setDebug("y", ourPose.getY());
            telemetry.setDebug("heading", ourPose.getHeading());
            //tobePowerRatio = Math.max(sonicHedgehogSensor.getDistance(DistanceUnit.CM) * tobeDistanceRatio,1);

            if (!lastAMash && gamepad1.a) {
                if (precisionMode) {
                    precisionMode = false;
                    precisionModifier = 1.2;
                    telemetry.setData("Precision Mode", "DEACTIVATED!");

                } else {
                    precisionMode = true;
                    precisionModifier = 0.5;
                    telemetry.setData("Precision Mode", "ACTIVATED!");

                }
                runtime.reset();
            }
            lastAMash = gamepad1.a;

            if (!lastBMash && gamepad1.b) {
                if (invertedControls == 1) {
                    invertedControls = -1;
                    telemetry.setData("Inverse Controls", "ACTIVATED!");
                } else if (invertedControls == -1) {
                    invertedControls = 1;
                    telemetry.setData("Inverse Controls", "DEACTIVATED");
                }
            }
            lastBMash = gamepad1.b;

            if (!lastXMash && gamepad2.x) {
                if (!wobbleGrabLock) {
                    wobbleGrabLock = true;
                    telemetry.setData("Wobble Grabber", "LOCKED");
                } else {
                    wobbleGrabLock = false;
                    telemetry.setData("Wobble Grabber", "UNLOCKED");
                }
            }

            double r = MoveUtils.joystickXYToRadius(-gamepad1.left_stick_x, -gamepad1.left_stick_y * invertedControls);
            double robotAngle = MoveUtils.joystickXYToAngle(-gamepad1.left_stick_x, gamepad1.left_stick_y * invertedControls);

            double[] powers = MoveUtils.theAlgorithm(r, robotAngle, gamepad1.right_stick_x, precisionModifier);
            MoveUtils.setEachMotor(new DcMotor[]{robot.left_drive, robot.right_drive, robot.left_drive_2, robot.right_drive_2}, powers);

            /*
            if(gamepad1.left_bumper) {
                intake.setPower(0);
            } else if (gamepad1.right_bumper) {
                intake.setPower(.8);
                //tobeFlywheel.setVelocity(2 * Math.PI * -tobeMaxEncoder / ticksPerRevolution, AngleUnit.RADIANS);
            }

            if (gamepad1.left_trigger > 0.2) {
                outtake.setPower(0);
            } else if (gamepad1.right_trigger > 0.2) {
                outtake.setPower(-0.9);
                // tobeFlywheel.setVelocity(2 * Math.PI * -tobeSpeedOneEncoder / ticksPerRevolution, AngleUnit.RADIANS);
            }
            */

            if (gamepad1.right_bumper) {
                intake.setPower(0.9);
            } else if (gamepad1.left_bumper) {
                intake.setPower(0);
            }

            if (gamepad1.right_trigger > 0.2) {
                kickFinished = false;
                outtake.setVelocity(-outTake75Speed, AngleUnit.RADIANS);
            }

            if (gamepad1.right_trigger > 0.2 && !kickFinished && kickTime.milliseconds() >= kickInterval) {
                kickStarting = true;
            }

            /*
            if (gamepad1.right_bumper) {
                kicker.setPower(1);
            } else if (gamepad1.left_bumper) {
                kicker.setPower(-1);
            } else {
                kicker.setPower(0);
            }
            */

            if (kickStarting && !kickFinished && Math.abs(outtake.getVelocity(AngleUnit.RADIANS) + outTake75Speed) < Math.pow(10, -2) * 5
                    /*&& drive.getPoseVelocity().getX() < 0.005  && drive.getPoseVelocity().getY() < 0.005
                    && drive.getPoseVelocity().getHeading() < 0.005*/) {

                if (kicks >= 5 && kickTime.milliseconds() >= kickInterval) {
                    kicker.setPower(0);
                    outtake.setVelocity(0);
                    kicks = 0;
                    kickFinished = true;
                    kickStarting = false;
                }

                if (kickTime.milliseconds() >= kickInterval) {
                    kickDirection = kickDirection * -1;
                    kicker.setPower(kickDirection);
                    kickTime.reset();
                    if (kickDirection == -1) {
                        kicks++;
                    }
                }

                if (kicks == 0) {
                    kickTime.reset();
                }

            }
            if (gamepad1.left_trigger > 0.3) {
                outtake.setVelocity(0);
                kicker.setPower(0);
                kickFinished = true;
                kickStarting = false;
            }

            //if (gamepad2.x) {
            //   tobeFlywheel.setPower(-tobePowerRatio);
            //}

            if (gamepad2.right_trigger > 0.05) {
                robot.wobbleArm.setPower(gamepad2.right_trigger);
            } else if (gamepad2.left_trigger > 0.05) {
                robot.wobbleArm.setPower(-gamepad2.left_trigger);
            } else robot.wobbleArm.setPower(0);
            if (gamepad2.right_bumper) {
                robot.wobbleGrab.setPower(1);
            } else if (gamepad2.left_bumper) {
                robot.wobbleGrab.setPower(-1);
            } else if (!wobbleGrabLock) robot.wobbleGrab.setPower(0);


            //telemetry.setData("raw ultrasonic", sonicHedgehogSensor.getDistance(DistanceUnit.CM));
            //telemetry.setData("cm", "%.2f cm", sonicHedgehogSensor.getDistance(DistanceUnit.CM));
            telemetry.setDebug("Outtake Actual Speed", outtake.getVelocity(AngleUnit.RADIANS));
            telemetry.setDebug("Outtake Intended Speed", outTake75Speed);
            telemetry.setDebug("Kick Speed", kicker.getPower());
            telemetry.setDebug("Kicks Completed", kicks);
            telemetry.setDebug("Kicking?", !kickFinished);
            telemetry.setDebug("kick interval", kickInterval);
            telemetry.setDebug("kickTime", kickTime);
            telemetry.setDebug("outtake error", Math.abs(outtake.getVelocity(AngleUnit.RADIANS) - outTake75Speed));
            telemetry.setDebug("error threshhold", Math.pow(10, -1) * 3);
            telemetry.setDebug("error difference", Math.pow(10, -1) - Math.abs(outtake.getVelocity(AngleUnit.RADIANS) - outTake75Speed));
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
