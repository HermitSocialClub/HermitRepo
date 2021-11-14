package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.T265LocalizerRR;
import org.hermitsocialclub.telecat.PersistantTelemetry;

@TeleOp(name = "Meet0Tele")
public class Meet0TeleOp extends OpMode {
    Canvas field;
    TelemetryPacket packet;
    DcMotor duck_wheel;

    private PersistantTelemetry telemetry;

    private BaselineMecanumDrive drive;

    private final FtcDashboard dash = FtcDashboard.getInstance();

    private final int robotRadius = 7;

    private double trigVal = 0;

    MotorConfigurationType liftType = MotorConfigurationType
            .getMotorType(NeveRest40Gearmotor.class);

    @Override

    public void init() {
        telemetry = new PersistantTelemetry(super.telemetry);
        drive = new BaselineMecanumDrive(hardwareMap, telemetry);
        drive.setPoseEstimate(new Pose2d(0,0));
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

        trigVal = gamepad1.left_trigger > 0.05 ? -gamepad1.left_trigger/10 :
                gamepad1.right_trigger > 0.05 ? gamepad1.right_trigger : 0.000005;

        drive.lift.setVelocity(liftType
                .getAchieveableMaxTicksPerSecond() * .15 *
                trigVal, AngleUnit.RADIANS);

        if (gamepad1.right_bumper) {
            drive.intake.setPower(-.85);
        } else if (gamepad1.left_bumper) {
            drive.intake.setPower(.85);
        } else drive.intake.setPower(0);

        if (gamepad1.x) {
            drive.duck_wheel.setPower(-0.3);
        }else{
            drive.duck_wheel.setPower(0);
        }
       /* drive.duck_wheel.setVelocity(liftType
                .getAchieveableMaxTicksPerSecond() * .65 *
                trigVal, AngleUnit.RADIANS);
*/
        packet = new TelemetryPacket();

        field = packet.fieldOverlay();

        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.right_stick_x,
                        -gamepad1.left_stick_x
                )
        );

        drive.update();

        Pose2d pose = drive.getPoseEstimate();
        double angle = pose.getHeading();
        /*telemetry.setData("x", pose.getX());
        telemetry.setData("y", pose.getY());
        telemetry.setData("heading", pose.getHeading());*/



        field.strokeCircle(pose.getX(),pose.getY(),angle);
        double arrowX = Math.cos(angle) * robotRadius, arrowY = Math.sin(angle) * robotRadius;
        double x1 = pose.getX() + arrowX  / 2, y1 = pose.getY() + arrowY / 2;
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
