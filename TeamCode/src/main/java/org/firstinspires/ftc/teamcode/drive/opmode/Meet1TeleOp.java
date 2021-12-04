package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.localizers.T265LocalizerRR;
import org.hermitsocialclub.telecat.PersistantTelemetry;

@TeleOp(name = "Meet1Tele")
public class Meet1TeleOp extends OpMode {
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private final int robotRadius = 7;
    Canvas field;
    TelemetryPacket packet;
    private PersistantTelemetry telemetry;
    private BaselineMecanumDrive drive;
    private double trigVal = 0;
    private final ElapsedTime duckTimer = new ElapsedTime();
    private boolean duckPress = false;
    private MotorConfigurationType duckType;
    private double duckSpeedRadians;
    private double duckSlow;
    private double duckFast;

    MotorConfigurationType liftType;
    private double liftSpeed = 10;


    @Override

    public void init() {
        telemetry = new PersistantTelemetry(super.telemetry);
        drive = new BaselineMecanumDrive(hardwareMap, telemetry);
        drive.setPoseEstimate(new Pose2d(0, 0));

        duckType = drive.duck_wheel.getMotorType();
        duckSpeedRadians = duckType.getAchieveableMaxRPMFraction() / 60 * duckType.getMaxRPM() * 2 * Math.PI;

        liftType = drive.lift.getMotorType();
        drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        /*
        trigVal = gamepad1.left_trigger > 0.05 ? -gamepad1.left_trigger / 10 :
                gamepad1.right_trigger > 0.05 ? gamepad1.right_trigger : 0.000005;

        drive.lift.setVelocity(liftType
                .getAchieveableMaxTicksPerSecond() * .15 *
                trigVal, AngleUnit.RADIANS);
        */
        drive.lift.setTargetPosition(drive.lift.getCurrentPosition() +
                (int)(liftSpeed * gamepad1.right_stick_y));
        drive.lift.setPower(.45);
        if (gamepad1.right_bumper) {
            drive.intake.setPower(.85);
        } else if (gamepad1.left_bumper) {
            drive.intake.setPower(-.85);
        } else drive.intake.setPower(0);

        if (gamepad1.right_stick_button) {
            duckPress = !duckPress;
            duckTimer.reset();
        }
        if (duckPress) {
            if (duckTimer.milliseconds() < 200) {
                drive.duck_wheel.setVelocity(duckSlow * duckSpeedRadians, AngleUnit.RADIANS);
            } else if (duckTimer.milliseconds() < 400) {
                drive.duck_wheel.setVelocity(duckFast * duckSpeedRadians, AngleUnit.RADIANS);
            } else if (duckTimer.milliseconds() >= 400) {
                drive.duck_wheel.setVelocity(0);
                duckPress = false;
                duckTimer.reset();
            }
        } else drive.duck_wheel.setVelocity(0);

        if (gamepad1.b) {
            drive.setWeightedDrivePowerFollower(new Pose2d(
                    0, -1, 0
            ));
        }
       /* drive.duck_wheel.setVelocity(liftType
                .getAchieveableMaxTicksPerSecond() * .65 *
                trigVal, AngleUnit.RADIANS);
*/
        packet = new TelemetryPacket();

        field = packet.fieldOverlay();
        if (!gamepad1.b) drive.setWeightedDrivePowerFollower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                ).times(.45)
        );
        drive.update();

        Pose2d pose = drive.getPoseEstimate();
        double angle = pose.getHeading();
        /*telemetry.setData("x", pose.getX());
        telemetry.setData("y", pose.getY());
        telemetry.setData("heading", pose.getHeading());*/


        field.strokeCircle(pose.getX(), pose.getY(), angle);
        double arrowX = Math.cos(angle) * robotRadius, arrowY = Math.sin(angle) * robotRadius;
        double x1 = pose.getX() + arrowX / 2, y1 = pose.getY() + arrowY / 2;
        double x2 = pose.getX() + arrowX, y2 = pose.getY() + arrowY;
        field.strokeLine(x1, y1, x2, y2);
        packet.put("Pose", pose.toString());
        packet.put("Pose Confidence", ((T265LocalizerRR) (drive.getLocalizer())).getConfidence());

        dash.sendTelemetryPacket(packet);

    }

    @Override
    public void stop() {
        super.stop();
    }
}
