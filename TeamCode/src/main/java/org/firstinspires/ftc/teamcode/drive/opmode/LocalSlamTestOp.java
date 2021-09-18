package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.hermitsocialclub.telecat.PersistantTelemetry;

@TeleOp(name = "Local Slam")
public class LocalSlamTestOp extends OpMode {
    Canvas field;
    TelemetryPacket packet;

    private PersistantTelemetry telemetry;

    private BaselineMecanumDrive drive;

    private final FtcDashboard dash = FtcDashboard.getInstance();

    private final int robotRadius = 7;

    @Override
    public void init() {
        telemetry = new PersistantTelemetry(super.telemetry);
        drive = new BaselineMecanumDrive(hardwareMap, telemetry);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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



        packet = new TelemetryPacket();

        field = packet.fieldOverlay();

        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
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

        dash.sendTelemetryPacket(packet);

    }

    @Override
    public void stop() {
        super.stop();
    }
}
