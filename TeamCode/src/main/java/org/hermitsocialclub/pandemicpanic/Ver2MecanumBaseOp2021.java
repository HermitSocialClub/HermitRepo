package org.hermitsocialclub.pandemicpanic;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.motors.GoBILDA5201Series;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Version 2 2021 Mecanum Base Op", group = "Hermit")

public class Ver2MecanumBaseOp2021 extends LinearOpMode {

    private PersistantTelemetry pt = new PersistantTelemetry(telemetry);
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
    private double ticksPerRevolution = MotorConfigurationType.getMotorType(GoBILDA5201Series.class).getTicksPerRev();
    private double tobeMaxEncoder = MotorConfigurationType.getMotorType(GoBILDA5201Series.class).getAchieveableMaxTicksPerSecond();;
    private double tobeSpeedThreeEncoder = tobeMaxEncoder * .5;
    private double tobeSpeedTwoEncoder = 0.6 * tobeMaxEncoder ;
    private double tobeSpeedOneEncoder = 0.25 * tobeMaxEncoder;
    private double tobeDistanceRatio = 0.0625; //based off observed distance of ring at max rpm
    private double tobePowerRatio;

    private boolean kickFinished = true;
    private long kickInterval = 300;
    private int kickDirection = -1;
    private int kicks = 0;


    MotorConfigurationType goBildaOuttake =  MotorConfigurationType.getMotorType(GoBILDA5202Series.class);
    private double outTake75Speed;
    private BaselineMecanumDrive drive;

    private final double WOBBLE_GRAB_INCREMENT = .02;

private boolean wobbleGrabLock = false;
private boolean lastXMash = false;
private boolean kickStarting = false;

    //private DistanceSensor sonicHedgehogSensor;
    private DcMotorEx intake;
    private DcMotorEx outtake;

    private CRServo kicker;



    @Override
    public void runOpMode() throws InterruptedException {

        intake = hardwareMap.get(DcMotorEx.class, "tobeFlywheel");
        kicker = hardwareMap.get(CRServo.class,"kicker");
        outtake = hardwareMap.get(DcMotorEx.class,"takeruFlyOut");
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.init(hardwareMap);
        drive = new BaselineMecanumDrive(hardwareMap,pt);
        drive.setPoseEstimate(PoseStorage.currentPose);
        pt.setDebug("Pose",drive.getPoseEstimate());
        pt.setDebug("intendedPose",PoseStorage.currentPose);

        outTake75Speed = -((SPEED_PERCENT * 2 * Math.PI * goBildaOuttake.getMaxRPM() * goBildaOuttake.getAchieveableMaxRPMFraction())/60);

        Trajectory[] trajs = {
                drive.trajectoryBuilder(new Pose2d(48,48,0),0)//0
                    .lineToConstantHeading(new Vector2d(-3,-36))
                    .build(),
                drive.trajectoryBuilder(new Pose2d(48,24,0),0)//1
                    .lineToConstantHeading(new Vector2d(-3,-36))
                    .build(),
                drive.trajectoryBuilder(new Pose2d(48,0,0),0)//2
                        .lineToConstantHeading(new Vector2d(-3,-36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(48,-24,0),0)//3
                        .lineToConstantHeading(new Vector2d(-3,-36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(48,-48,0),0)//4
                        .lineToConstantHeading(new Vector2d(-3,-36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(24,48,0),0)//5
                        .lineToConstantHeading(new Vector2d(-3,-36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(24,24,0),0)//6
                        .lineToConstantHeading(new Vector2d(-3,-36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(24,0,0),0)//7
                        .lineToConstantHeading(new Vector2d(-3,-36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(24,-24,0),0)//8
                        .lineToConstantHeading(new Vector2d(-3,-36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(24,-48,0),0)//9
                        .lineToConstantHeading(new Vector2d(-3,-36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(0,48,0),0)//10
                        .lineToConstantHeading(new Vector2d(-3,-36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(0,24,0),0)//11
                        .lineToConstantHeading(new Vector2d(-3,-36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(0,0,0),0)//12
                        .lineToConstantHeading(new Vector2d(-3,-36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(0,-24,0),0)//16
                        .lineToConstantHeading(new Vector2d(-3,-36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(0,-48,0),0)//17
                        .lineToConstantHeading(new Vector2d(-3,-36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(-24,48,0),0)//18
                        .lineToConstantHeading(new Vector2d(-3,-36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(-24,24,0),0)//19
                        .lineToConstantHeading(new Vector2d(-3,-36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(-24,0,0),0)//20
                        .lineToConstantHeading(new Vector2d(-3,-36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(-24,-24,0),0)//21
                        .lineToConstantHeading(new Vector2d(-3,-36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(-24,-48,0),0)//22
                        .lineToConstantHeading(new Vector2d(-3,-36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(-48,48,0),0)//23
                        .lineToConstantHeading(new Vector2d(-3,-36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(-48,24,0),0)//24
                        .lineToConstantHeading(new Vector2d(-3,-36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(-48,0,0),0)//25
                        .lineToConstantHeading(new Vector2d(-3,-36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(-48,-24,0),0)//26
                        .lineToConstantHeading(new Vector2d(-3,-36))
                        .build(),
                drive.trajectoryBuilder(new Pose2d(-48,-48,0),0)//27
                        .lineToConstantHeading(new Vector2d(-3,-36))
                        .build(),
        };

        pt.setDebug("paths","done");
        pt.setData("Inverse Controls", "DEACTIVATED");
        pt.setData("Precision Mode", "DEACTIVATED!");
        pt.setData("Wobble Grabber","UNLOCKED");



        //sonicHedgehogSensor = hardwareMap.get(DistanceSensor.class,"Sonic the Hedgehog");
        //tobePowerRatio = Math.max(sonicHedgehogSensor.getDistance(DistanceUnit.CM) * tobeDistanceRatio,1);


        waitForStart();
        //initialLeftTicks = robot.leftEncoder.getCurrentPosition();
        //initialRightTicks = robot.rightEncoder.getCurrentPosition();
        //initialTopTicks = robot.frontEncoder.getCurrentPosition();
        telemetry.speak("Hola. Cómo estás?", "spa", "mx");
        telemetry.update();

        while (opModeIsActive()) {
            // Make sure to call drive.update() on *every* loop
            // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy
            drive.update();

            // Retrieve your pose
            Pose2d myPose = drive.getPoseEstimate();

            if(gamepad1.right_stick_button){
                double minDist = Math.sqrt(Math.pow((myPose.getX() - trajs[0].start().getX()),2)
                        + Math.pow((myPose.getY() - trajs[0].start().getY()),2));
                int closest = 0;
                for(int i = 1; i < trajs.length; i++){
                    double dist = Math.sqrt(Math.pow((myPose.getX() - trajs[i].start().getX()),2)
                            + Math.pow((myPose.getY() - trajs[i].start().getY()),2));
                    if(dist < minDist){
                        minDist = dist;
                        closest = i;
                    }
                }
                drive.followTrajectory(trajs[closest]);
            }
            pt.setDebug("x", myPose.getX());
            pt.setDebug("y", myPose.getY());
            pt.setDebug("heading", myPose.getHeading());
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
            if (!lastXMash && gamepad2.x){
                if(!wobbleGrabLock){
                    wobbleGrabLock = true;
                    pt.setData("Wobble Grabber","LOCKED");
                }else {
                    wobbleGrabLock = false;
                    pt.setData("Wobble Grabber","UNLOCKED");
                }
            }

            double r = MoveUtils.joystickXYToRadius(-gamepad1.left_stick_x, -gamepad1.left_stick_y * invertedControls);
            double robotAngle = MoveUtils.joystickXYToAngle(-gamepad1.left_stick_x, gamepad1.left_stick_y * invertedControls);

            double[] powers = MoveUtils.theAlgorithm(r, robotAngle, gamepad1.right_stick_x, precisionModifier);
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

            if (gamepad1.right_bumper){
                intake.setPower(0.9);
            }else if (gamepad1.left_bumper){
                intake.setPower(0);
            }

            if (gamepad1.right_trigger > 0.2){
                kickFinished = false;
                outtake.setVelocity(-outTake75Speed,AngleUnit.RADIANS);
            }

            if(gamepad1.right_trigger > 0.2 && !kickFinished  && kickTime.milliseconds() >= kickInterval){
                kickStarting = true;
            }

            /*if(gamepad1.right_bumper){
                kicker.setPower(1);
            }else if(gamepad1.left_bumper){
                kicker.setPower(-1);
            }else kicker.setPower(0);*/

            if(kickStarting && !kickFinished  && Math.abs(outtake.getVelocity(AngleUnit.RADIANS)+outTake75Speed) < Math.pow(10,-2)*5
                    /*&& drive.getPoseVelocity().getX() < .005  && drive.getPoseVelocity().getY() < .005
                    && drive.getPoseVelocity().getHeading() <.005*/){

                if(kicks >= 5 && kickTime.milliseconds() >= kickInterval){
                    kicker.setPower(0);
                    outtake.setVelocity(0);
                    kicks = 0;
                    kickFinished = true;
                    kickStarting = false;
                }

                if(kickTime.milliseconds() >= kickInterval){
                    kickDirection = kickDirection * -1;
                    kicker.setPower(kickDirection);
                    kickTime.reset();
                    if(kickDirection == -1){
                        kicks++;
                    }
                }
                if(kicks == 0){
                    kickTime.reset();
                }

            }
            if(gamepad1.left_trigger > .3){
                outtake.setVelocity(0);
                kicker.setPower(0);
                kickFinished = true;
                kickStarting = false;
            }

            //if (gamepad2.x){
                //tobeFlywheel.setPower(-tobePowerRatio);
           // }

            if(gamepad2.right_trigger > .05){
                robot.wobbleArm.setPower(gamepad2.right_trigger);
            }else if(gamepad2.left_trigger > .05){
                robot.wobbleArm.setPower( -gamepad2.left_trigger);
            } else robot.wobbleArm.setPower(0);
            if(gamepad2.right_bumper){
                robot.wobbleGrab.setPower(1);
            } else if(gamepad2.left_bumper){
                robot.wobbleGrab.setPower(-1);
            }else if(!wobbleGrabLock) robot.wobbleGrab.setPower(0);



            //pt.setData("raw ultrasonic", sonicHedgehogSensor.getDistance(DistanceUnit.CM));
            //pt.setData("cm", "%.2f cm", sonicHedgehogSensor.getDistance(DistanceUnit.CM));
            pt.setDebug("Outtake Actual Speed",outtake.getVelocity(AngleUnit.RADIANS));
            pt.setDebug("Outtake Intended Speed", outTake75Speed);
            pt.setDebug("Kick Speed",kicker.getPower());
            pt.setDebug("Kicks Completed",kicks);
            pt.setDebug("Kicking?", !kickFinished);
            pt.setDebug("kick interval",kickInterval);
            pt.setDebug("kickTime",kickTime);
            pt.setDebug("outtake error",Math.abs(outtake.getVelocity(AngleUnit.RADIANS)-outTake75Speed));
            pt.setDebug("error threshhold",Math.pow(10,-1)*3);
            pt.setDebug("error difference", Math.pow(10,-1) - Math.abs(outtake.getVelocity(AngleUnit.RADIANS)-outTake75Speed));

        }

    }
public void kickerTime (long time, double power){
    long beginning = System.currentTimeMillis();
    long end = beginning + time;
    while ( end > System.currentTimeMillis()){
        kicker.setPower(power);
    }
    }


}
