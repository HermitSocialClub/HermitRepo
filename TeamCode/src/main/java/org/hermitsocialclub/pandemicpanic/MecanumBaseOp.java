package org.hermitsocialclub.pandemicpanic;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.hermitsocialclub.telecat.PersistantTelemetry;
import org.hermitsocialclub.util.MoveUtils;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Mecanum Base Op", group = "Hermit")

public class MecanumBaseOp extends LinearOpMode {

    private final PersistantTelemetry pt = new PersistantTelemetry(telemetry);
    MecanumConfiguration robot = new MecanumConfiguration();
    ElapsedTime runtime = new ElapsedTime();
    private boolean lastAMash = false;
    private boolean lastBMash = false;
    public boolean precisionMode = false;
    public double precisionModifier = 1.25;
    public double invertedControls = 1;
    double clamperPosition = 0;
    double topClawPosition = 0;
    private boolean foundationPull = false;
    private double initialLeftTicks, initialRightTicks, initialTopTicks;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();
        initialLeftTicks = robot.leftEncoder.getCurrentPosition();
        initialRightTicks = robot.rightEncoder.getCurrentPosition();
        initialTopTicks = robot.frontEncoder.getCurrentPosition();


        telemetry.speak("Hola. Cómo estás?", "spa", "mx");

        while (opModeIsActive()) {

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

            if (gamepad1.right_bumper) {
                clampinator(0);
            }
            if (gamepad1.left_bumper) {
                clampinator(1);
            }

            pt.setDebug("CLAMP_POSITION", clamperPosition);

            if (Math.abs(gamepad2.right_stick_y) > 0.05) {
                linear(-gamepad2.right_stick_y);
                if (gamepad2.right_stick_y < 0) foundationPull = true;
            } else {
                linear(0);
            }

            if (Math.abs(gamepad2.right_trigger) >= 0.05 || Math.abs(gamepad1.right_trigger) >= 0.05) {
                clamperPosition += 0.05;
                clamperPosition = Math.min(Math.max(clamperPosition, 0.2), 0.8);

                clampinator(clamperPosition);
            }
            if (Math.abs(gamepad2.left_trigger) >= 0.05 || Math.abs(gamepad2.left_trigger) >= 0.05) {
                clamperPosition -= 0.05;
                clamperPosition = Math.min(Math.max(clamperPosition, 0.2), 0.8);
                clampinator(clamperPosition);
            }

            if (gamepad2.right_bumper) {
                foundationPull = false;
                topClawPosition += 0.075;
                topClawPosition = Math.min(Math.max(topClawPosition, 0.2), 1);
                robot.topClaw.setPosition(topClawPosition);
            }
            if (gamepad2.left_bumper) {
                foundationPull = false;
                topClawPosition -= 0.075;
                topClawPosition = Math.min(Math.max(topClawPosition, 0.2), 1);
                robot.topClaw.setPosition(topClawPosition);
            }

            if (foundationPull) {
                topClawPosition = 0.2;
                robot.topClaw.setPosition(topClawPosition);
            }
            if (gamepad1.x) {
                robot.spinner.setPower(1);
            } else if (gamepad1.y) {
                robot.spinner.setPower(.75);
            } else if (gamepad1.a) {
                robot.spinner.setPower(.5);
            } else if (gamepad1.b) {
                robot.spinner.setPower(.25);
            }
            pt.setDebug("leftEncoder ticks", robot.leftEncoder.getCurrentPosition() - initialLeftTicks);
            pt.setDebug("leftEncoder velocity", robot.leftEncoder.getVelocity());
            pt.setDebug("rightEncoder ticks", robot.rightEncoder.getCurrentPosition() - initialRightTicks);
            pt.setDebug("rightEncoder velocity", robot.rightEncoder.getVelocity());
            pt.setDebug("topEncoder ticks", robot.frontEncoder.getCurrentPosition() - initialTopTicks);
            pt.setDebug("topEncoder velocity", robot.frontEncoder.getVelocity());
        }

    }

    public void linear(double position) {
        robot.arm.setPower(position);
        robot.arm2.setPower(-position);
    }

    public void clampinator(double positionPewPew) {
        robot.block_Clamper.setPosition(positionPewPew);
        robot.block_Clamper_2.setPosition(Math.abs(positionPewPew - 0.8));
    }

}
