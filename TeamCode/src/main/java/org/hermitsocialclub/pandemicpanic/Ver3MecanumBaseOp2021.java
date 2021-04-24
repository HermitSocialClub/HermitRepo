package org.hermitsocialclub.pandemicpanic;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.opmode.BaselineMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;
import org.hermitsocialclub.telecat.PersistantTelemetry;


@TeleOp(name = "Version 3 2021 Mecanum Base Op", group = "Hermit")
public class Ver3MecanumBaseOp2021 extends LinearOpMode {

    private static final double WOBBLE_GRAB_INCREMENT = .02;
    public static double DRAWING_TARGET_RADIUS = 2;
    public static double SPEED_PERCENT = 0.75;
    public static double POWER_PERCENT = 0.8;
    private static double INTAKE_PERCENT = .25;
    private double lastIntake = 0;
    private double currentIntake;
    private static double INTAKE_GEAR = 1;
    public static double NEAR_ZERO_THRESHHOLD = Math.pow(10, -1) * 1.5;
    private final PersistantTelemetry pt = new PersistantTelemetry(telemetry);
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime kickTime = new ElapsedTime();
    private final Vector2d powerLaunchVector = new Vector2d(-14, 12.50);
    private final MotorConfigurationType goBildaOuttake = MotorConfigurationType.getMotorType(GoBILDA5202Series.class);
    private final MotorConfigurationType goBildaIntake = MotorConfigurationType.getMotorType(GoBILDA5202Series.class);
    private final Mode mode = Mode.NORMAL_CONTROL;
    private final PIDFController headingController = new PIDFController(BaselineMecanumDrive.HEADING_PID);
    private final ElapsedTime ringTime = new ElapsedTime();
    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing
    private final Vector2d targetPosition = new Vector2d(0, 0);
    private final Vector2d shootingPosition = new Vector2d(-3, -41);
    private final double shootingHeading = Math.toRadians(-15);
    public boolean precisionMode = false;
    public double precisionModifier = 1.25;
    public double invertedControls = 1;
    private boolean lastAMash = false;
    private boolean lastBMash = false;
    private boolean lastDownMash = false;
    private boolean lastUpMash = false;
    private boolean lastLeftMash = false;
    private boolean lastRTriggerMash = false;
    private boolean intakeOn = false;
    private double powerShotSpeed;
    private boolean kickFinished = true;
    private boolean kickDirection = false;
    private int kicks = 0;
    private BaselineMecanumDrive drive;
    private boolean wobbleGrabLock = false;
    private boolean lastXMash = false;
    private boolean kickStarting = false;
    private boolean lastYMash = false;
    private boolean alwaysOn = true;
    private boolean hopperMash = false;
    private boolean ringDetected = false;
    private double HOPPER_POSITION = 1;
    private boolean last2XMash = false;

    @Override
    public void runOpMode() throws InterruptedException {
        double maxKicks = 5;
        this.drive = new BaselineMecanumDrive(hardwareMap, pt);
        this.drive.setPoseEstimate(PoseStorage.currentPose);
        this.pt.setDebug("Pose", drive.getPoseEstimate());
        this.pt.setDebug("intendedPose", PoseStorage.currentPose);

        double outTake75Speed = -((SPEED_PERCENT * 2 * Math.PI * goBildaOuttake.getMaxRPM() * goBildaOuttake.getAchieveableMaxRPMFraction()) / 60);
        powerShotSpeed = -((POWER_PERCENT * 2 * Math.PI * goBildaOuttake.getMaxRPM() * goBildaOuttake.getAchieveableMaxRPMFraction()) / 60);
        double intake85Speed = -((INTAKE_PERCENT * 2 * Math.PI * goBildaIntake.getMaxRPM() * goBildaIntake.getAchieveableMaxRPMFraction() * INTAKE_GEAR) / 60);

        headingController.setInputBounds(-Math.PI, Math.PI);
        drive.kicker.setPosition(0.3);
        Trajectory constantLaunchSpline2 = drive.trajectoryBuilder(new Pose2d(powerLaunchVector, 0))
                .splineToConstantHeading(new Vector2d(-3, -24.50), 0)
                .addSpatialMarker(new Vector2d(0, -2.50), () -> launchRing(1, powerShotSpeed))
                .addSpatialMarker(new Vector2d(0, -14.50), () -> launchRing(1, powerShotSpeed))
                .addSpatialMarker(new Vector2d(0, -20.50), () -> launchRing(1, powerShotSpeed))
                .build();
        Trajectory t00 = drive.trajectoryBuilder(new Pose2d(48, 0, 0), 0)//0 (0,0)
                .lineToConstantHeading(shootingPosition)
                .build();
        Trajectory t01 = drive.trajectoryBuilder(new Pose2d(48, -24, 0), 0)//1 (0,1)
                .lineToConstantHeading(shootingPosition)
                .build();
        Trajectory t10 = drive.trajectoryBuilder(new Pose2d(24, 0, 0), 0)//3 (1,0)
                .lineToConstantHeading(shootingPosition)
                .build();
        Trajectory t11 = drive.trajectoryBuilder(new Pose2d(24, -24, 0), 0)//4 (1,1)
                .lineToConstantHeading(shootingPosition)
                .build();
        Trajectory t20 = drive.trajectoryBuilder(new Pose2d(0, 0, 0), 0)//6 (2,0)
                .lineToConstantHeading(shootingPosition)
                .build();
        Trajectory t21 = drive.trajectoryBuilder(new Pose2d(0, -24, 0), 0)//7 (2,1)
                .lineToConstantHeading(shootingPosition)
                .build();
        //takes about 3.76 seconds to generate, about a quarter second each
        Trajectory[][] traj = {{
                t00,
                t01,
                drive.trajectoryBuilder(new Pose2d(48, -48, 0), 0)//2 (0,2)
                        .lineToConstantHeading(shootingPosition)
                        .build()},
                {t10,
                        t11,
                        drive.trajectoryBuilder(new Pose2d(24, -48, 0), 0)//5 (1,2)
                                .lineToConstantHeading(shootingPosition)
                                .build()},
                {t20,
                        t21,
                        drive.trajectoryBuilder(new Pose2d(0, -48, 0), 0)//8 (2,2)
                                .lineToConstantHeading(shootingPosition)
                                .build()},
                {drive.trajectoryBuilder(new Pose2d(-24, 0, 0), 0)//9 (3,0)
                        .lineToConstantHeading(shootingPosition)
                        .build(),
                        drive.trajectoryBuilder(new Pose2d(-24, -24, 0), 0)//10 (3,1)
                                .lineToConstantHeading(shootingPosition)
                                .build(),
                        drive.trajectoryBuilder(new Pose2d(-24, -48, 0), 0)//11 (3,2)
                                .lineToConstantHeading(shootingPosition)
                                .build()},
                {drive.trajectoryBuilder(new Pose2d(-48, 0, 0), 0)//12 (4,0)
                        .lineToConstantHeading(shootingPosition)
                        .build(),
                        drive.trajectoryBuilder(new Pose2d(-48, -24, 0), 0)//13 (4,1)
                                .lineToConstantHeading(shootingPosition)
                                .build(),
                        drive.trajectoryBuilder(new Pose2d(-48, -48, 0), 0)//14 (4,2)
                                .lineToConstantHeading(shootingPosition)
                                .build()}
        };
        Trajectory[][] hiRes = {
                {
                        t00,  //00
                        drive.trajectoryBuilder(new Pose2d(48, -12, 0), 0)//0 01
                                .lineToConstantHeading(shootingPosition)
                                .build(),
                        t01 //02
                },
                {
                        drive.trajectoryBuilder(new Pose2d(36, 0, 0), 0)//3 10
                                .lineToConstantHeading(shootingPosition)
                                .build(),
                        drive.trajectoryBuilder(new Pose2d(36, -12, 0), 0)//4 11
                                .lineToConstantHeading(shootingPosition)
                                .build(),
                        drive.trajectoryBuilder(new Pose2d(36, -24, 0), 0)//5 12
                                .lineToConstantHeading(shootingPosition)
                                .build()
                },
                {
                        t10,//20
                        drive.trajectoryBuilder(new Pose2d(24, -12, 0), 0)//6 21
                                .lineToConstantHeading(shootingPosition)
                                .build(),
                        t11//22
                },
                {
                        drive.trajectoryBuilder(new Pose2d(12, 0, 0), 0)//7 30
                                .lineToConstantHeading(shootingPosition)
                                .build(),
                        drive.trajectoryBuilder(new Pose2d(12, -12, 0), 0)//8 31
                                .lineToConstantHeading(shootingPosition)
                                .build(),
                        drive.trajectoryBuilder(new Pose2d(12, -12, 0), 0)//9  32
                                .lineToConstantHeading(shootingPosition)
                                .build()
                },
                {
                        t20, // 40
                        drive.trajectoryBuilder(new Pose2d(0, -12, 0), 0)//10 41
                                .lineToConstantHeading(shootingPosition)
                                .build(),
                        t21//42
                }
        };

        pt.setDebug("paths", "done");
        pt.setData("Inverse Controls", "DEACTIVATED");
        pt.setData("Precision Mode", "DEACTIVATED!");
        pt.setData("Wobble Grabber", "UNLOCKED");
        pt.setData("Outtake", "Turns Off");

        waitForStart();
        drive.hopperLift.setPosition(.85);
        drive.kicker.setPosition(0);
        drive.wobbleGrab.setPosition(1);
        runtime.reset();
        //drive.friend.setPower(1);

        while (opModeIsActive()) {
            boolean justPressed = false;
            // Make sure to call drive.update() on *every* loop
            // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy
            drive.update();

            // Retrieve **our** pose
            Pose2d ourPose = drive.getPoseEstimate();

            //Context-Optimized Greedy Nearest Neighbor Search Algorithm
            if (gamepad1.right_stick_button) {
                if (runtime.seconds() < 90) { //check whether the robot is in endgame and needs to switch to shooting the power shots
                    double x = ourPose.getX();
                    double y = ourPose.getY();
                    //find the distance between the most likely trajectory and the robot's current position
                    double dist = (x - traj[2][0].end().getX()) * (x - traj[2][0].end().getX()) + (y - traj[2][0].end().getY()) * (y - traj[2][0].end().getY());
                    boolean bestDistFound = false;
                    Trajectory[][] trajectory;
                    int rowIndex = 2;
                    int colIndex = 0;
                    //if within the correct bounds, run using the more focused, higher resolution
                    if ((x < 60 && x > -12) && (y < -12 && y > -36)) {
                        trajectory = hiRes;
                        rowIndex = 4;
                    } else trajectory = traj;
                    //if the distance between the closest point and the robot is negligible, skip the search and run that trajectory
                    if (dist > NEAR_ZERO_THRESHHOLD) {
                        while (!bestDistFound) {
                            int tempRow = rowIndex;
                            int tempCol = colIndex;
                            //check if the node below the current node is closer than the current distance, if so set the index to this and the distance to
                            //the checked distance
                            if (rowIndex - 1 > -1) {
                                double checkDist = (x - trajectory[rowIndex - 1][colIndex].end().getX()) * (x - trajectory[rowIndex - 1][colIndex].end().getX())
                                        + (y - trajectory[rowIndex - 1][colIndex].end().getY()) * (y - trajectory[rowIndex - 1][colIndex].end().getY());
                                if (dist > checkDist) {
                                    dist = checkDist;
                                    tempRow = rowIndex - 1;
                                }
                            }
                            //check if the node above the current node is closer than the current distance, if so set the index to this and the distance to
                            //the checked distance
                            if (rowIndex + 1 < trajectory.length) {
                                double checkDist = (x - trajectory[rowIndex + 1][colIndex].end().getX()) * (x - trajectory[rowIndex + 1][colIndex].end().getX())
                                        + (y - trajectory[rowIndex + 1][colIndex].end().getY()) * (y - trajectory[rowIndex + 1][colIndex].end().getY());
                                if (dist > checkDist) {
                                    dist = checkDist;
                                    tempRow = rowIndex + 1;
                                }
                            }
                            //check if the node left of the current node is closer than the current distance, if so set the index to this and the distance to
                            //the checked distance
                            if (colIndex - 1 > -1) {
                                double checkDist = (x - trajectory[rowIndex][colIndex - 1].end().getX()) * (x - trajectory[rowIndex][colIndex - 1].end().getX())
                                        + (y - trajectory[rowIndex][colIndex - 1].end().getY()) * (y - trajectory[rowIndex][colIndex - 1].end().getY());
                                if (dist > checkDist) {
                                    dist = checkDist;
                                    tempRow = rowIndex;
                                    tempCol = colIndex - 1;
                                }
                            }
                            //check if the node right of the current node is closer than the current distance, if so set the index to this and the distance to
                            //the checked distance
                            if (colIndex + 1 < trajectory[rowIndex].length) {
                                double checkDist = (x - trajectory[rowIndex][colIndex + 1].end().getX()) * (x - trajectory[rowIndex][colIndex + 1].end().getX())
                                        + (y - trajectory[rowIndex][colIndex + 1].end().getY()) * (y - trajectory[rowIndex][colIndex + 1].end().getY());
                                if (dist > checkDist) {
                                    dist = checkDist;
                                    tempRow = rowIndex;
                                    tempCol = colIndex + 1;
                                }
                            }
                            //check whether any of the indexes were changed to a closer changed index, if so repeat with the closer index, if not end the
                            //search at the main node.
                            if (rowIndex != tempRow || colIndex != tempCol) {
                                rowIndex = tempRow;
                                colIndex = tempCol;
                            } else bestDistFound = true;
                        }
                    }
                    pt.setDebug("closest X: ", trajectory[rowIndex][colIndex].end().getX());
                    pt.setDebug("closest Y: ", trajectory[rowIndex][colIndex].end().getY());
                    //navigate along the closest path
                    drive.followTrajectory(trajectory[rowIndex][colIndex]);
                    //take steps to begin the outtake sequence
                    kickFinished = false;
                    kickTime.reset();
                    drive.hopperLift.setPosition(.375);
                    kicks = 0;
                    if (runtime.seconds() > 90) {
                        drive.outtake.setVelocity(-powerShotSpeed, AngleUnit.RADIANS);
                    } else {
                        drive.outtake.setVelocity(-outTake75Speed, AngleUnit.RADIANS);
                    }
                    justPressed = true;

                } else {
                    Trajectory toLaunch = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineToLinearHeading(constantLaunchSpline2.start(), 0)
                            .build();
                    drive.followTrajectory(toLaunch);
                    drive.followTrajectory(constantLaunchSpline2);
                }
            }
            //tobePowerRatio = Math.max(sonicHedgehogSensor.getDistance(DistanceUnit.CM) * tobeDistanceRatio,1);

           /* if (drive.color.getDistance(DistanceUnit.INCH) < 1.3) {
                ringDetected = true;
                ringTime.reset();
            }

            long ringInterval = 200;
            if (ringTime.milliseconds() < ringInterval && ringDetected) {
                if (drive.color.getDistance(DistanceUnit.INCH) > 1.3) {
                    ringDetected = false;
                }
            }

            if (ringTime.milliseconds() >= ringInterval && ringDetected) {
                ringDetected = false;
                kickFinished = false;
                drive.hopperLift.setPosition(.38);
                kickTime.reset();
                if (runtime.seconds() > 90) {
                    drive.outtake.setVelocity(-powerShotSpeed, AngleUnit.RADIANS);
                } else {
                    drive.outtake.setVelocity(-outTake75Speed, AngleUnit.RADIANS);
                }

            }*/

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
                    pt.setData("Wobble Grabber", "LOCKED");
                } else {
                    wobbleGrabLock = false;
                    pt.setData("Wobble Grabber", "UNLOCKED");
                }
            }
            lastXMash = gamepad1.square;

            // Declare a drive direction
            // Pose representing desired x, y, and angular velocity
            Pose2d driveDirection = new Pose2d();


                    // Read pose
                    Pose2d poseEstimate = ourPose;

                    // Create a vector from the gamepad x/y inputs
                    // Then, rotate that vector by the inverse of that heading
                    Vector2d input = new Vector2d(
                            -antiDeadzone(gamepad1.left_stick_y),
                            -antiDeadzone(gamepad1.left_stick_x)
                    );
                    driveDirection = new Pose2d(
                            input.getX() * precisionModifier * invertedControls,
                            input.getY() * precisionModifier * invertedControls,
                            -antiDeadzone(gamepad1.right_stick_x) * precisionModifier * invertedControls *.75
                    );

            // Draw bot on canvas

            drive.setWeightedDrivePower(driveDirection);

            // Update the heading controller with our current heading
            headingController.update(ourPose.getHeading());

            // Send telemetry packet off to dashboard

            if (gamepad1.right_bumper) {
                //drive.intake.setVelocity(intake85Speed,AngleUnit.RADIANS);
                drive.intake.setPower(.25);
                currentIntake = intake85Speed;
                intakeOn = true;
            } else if (gamepad1.left_bumper) {
                drive.intake.setVelocity(0);
                intakeOn = false;
            }
            /*if(Math.abs(drive.intake.getVelocity()) - Math.abs(lastIntake) < -.01 && intakeOn){
                currentIntake *= .99;
                drive.intake.setVelocity(currentIntake);
            }*/
            lastIntake = drive.intake.getVelocity();

            if (gamepad1.right_trigger > 0.02 && kickFinished) {
                kickFinished = false;
                kickTime.reset();
                drive.hopperLift.setPosition(.375);
                kicks = 0;
                if (runtime.seconds() > 90) {
                    drive.outtake.setVelocity(-powerShotSpeed, AngleUnit.RADIANS);
                } else {
                    drive.outtake.setVelocity(-outTake75Speed, AngleUnit.RADIANS);
                }
                justPressed = true;
            }


            if (gamepad1.right_trigger > 0.02 && !kickFinished && !lastRTriggerMash && !justPressed) {
                if (runtime.seconds() > 90) {
                    maxKicks = 6;
                } else {
                    maxKicks = 6;
                }
                kickStarting = true;
            }
            lastRTriggerMash = (gamepad1.right_trigger > 0.02);

            long kickInterval = 600;
            if (kickStarting && !kickFinished &&
                    Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) + ((runtime.seconds() > 90) ?
                            powerShotSpeed : outTake75Speed)) < NEAR_ZERO_THRESHHOLD) {
                if (kicks >= maxKicks && kickTime.milliseconds() >= kickInterval
                        && Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) + ((runtime.seconds() > 90) ?
                        powerShotSpeed : outTake75Speed)) < NEAR_ZERO_THRESHHOLD) {
                    drive.kicker.setPosition(1);
                    drive.hopperLift.setPosition(.85);
                    if (!alwaysOn && runtime.seconds() > 90) {
                        drive.outtake.setVelocity(0); }
                    kicks = 0;
                    kickFinished = true;
                    kickStarting = false;
                }
                if (kickTime.milliseconds() >= kickInterval || kicks == 0) {
                    if (kickDirection) {
                        kickDirection = false;
                        drive.kicker.setPosition(1);
                    } else {
                        kickDirection = true;
                        drive.kicker.setPosition(-1);
                    }kickTime.reset();
                    if (kickDirection) {
                        kicks++;
                    } }

                if (kicks == 0) {
                    kickTime.reset();
                }

            }
            if (gamepad1.left_trigger > 0.3) {
                drive.outtake.setVelocity(0);
                drive.kicker.setPosition(0);
                drive.hopperLift.setPosition(.85);
                kicks = 0;
                kickTime.reset();
                kickFinished = true;
                kickStarting = false;
            }

            if (!lastLeftMash && gamepad1.dpad_left) {
                if (drive.intake.getPower() != 0) {
                    drive.intake.setPower(0);
                } else if (drive.intake.getPower() == 0) {
                    //drive.intake.setVelocity(-intake85Speed,AngleUnit.RADIANS);
                    drive.intake.setPower(-.25);
                }
            }
            lastLeftMash = gamepad1.dpad_left;

            if (gamepad2.right_trigger > 0.02) {
                drive.friend.setPower(gamepad2.right_trigger);
            } else if (gamepad2.left_trigger > 0.02) {
                drive.friend.setPower(-gamepad2.left_trigger);
            }else{
                drive.friend.setPower(0);
            }
            //pt.setDebug("Hopper Position",drive.hopperLift.getPosition());
            if (Math.abs(gamepad2.left_stick_y) > .02) {
                drive.wobbleArm.setPower(antiDeadzone(gamepad2.left_stick_y) * .35);
            } else if (Math.abs(gamepad2.left_stick_y) < .05) {
                drive.wobbleArm.setPower(0);
            }
            if (gamepad2.right_bumper) {
                drive.wobbleGrab.setPosition(1);
            } else if (gamepad2.left_bumper) {
                drive.wobbleGrab.setPosition(-1);
            }
            if(gamepad2.x && !last2XMash){
                if(drive.intakeThirdStage.getPower() > 0.02){
                    drive.intakeThirdStage.setPower(0);
                }else if(drive.intakeThirdStage.getPower() < 0.02){
                    drive.intakeThirdStage.setPower(1);
                }
            }
            last2XMash = gamepad2.x;

            if(Math.abs(gamepad2.right_stick_y) > 0.2){
                drive.hook.setPower(antiDeadzone(gamepad2.right_stick_y));
            }else {
                drive.hook.setPower(0);
            }

            //TODO: Remove Telemetry when done figuring out problem with kicker
            pt.setDebug("Intake Velocity",drive.intake.getVelocity(AngleUnit.RADIANS));
            /*
            pt.setDebug("Outtake Actual Speed", drive.outtake.getVelocity(AngleUnit.RADIANS));
            pt.setDebug("Outtake Intended Speed", outTake75Speed);
            pt.setDebug("Kick Speed", drive.kicker.getPosition());
            pt.setDebug("Kicks Completed", kicks);
            pt.setDebug("Kicking?", !kickFinished);
            pt.setDebug("kick interval", kickInterval);
            pt.setDebug("kickTime", kickTime);
            pt.setDebug("outtake error", Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) + outTake75Speed));
            pt.setDebug("error threshhold", POWER_THRESHHOLD);
            pt.setDebug("error difference", POWER_THRESHHOLD - Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) + outTake75Speed));
            */
        }

    }

    public double antiDeadzone(double input) {
        return input;//(Math.copySign(Math.max(Math.abs(input) * (1.0 / .8) - .2, 0), input));
    }

    private void launchRing(int ringsToFire, double speed) {

        drive.outtake.setVelocity(speed, AngleUnit.RADIANS);
        while (opModeIsActive() && Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) - speed) > Math.pow(10, -1)) {

        }
        int ringsFired = 0;
        while (opModeIsActive() && ringsFired < ringsToFire) {
            pt.setDebug("ringsFired", ringsFired);
            drive.kicker.setPosition(.7);
            sleep(200);
            drive.kicker.setPosition(.3);
            while (Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) - speed) > Math.pow(10, -1)) {
            }
            ringsFired++;
        }
        drive.outtake.setVelocity(0);

    }

    enum Mode {
        NORMAL_CONTROL, ALIGN_TO_POINT
    }


}
