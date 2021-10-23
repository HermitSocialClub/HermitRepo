package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.hermitsocialclub.telecat.PersistantTelemetry;

import java.util.Objects;

/**
 * This routine is designed to calculate the maximum angular velocity your bot can achieve under load.
 * <p>
 * Upon pressing start, your bot will turn at max power for RUNTIME seconds.
 * <p>
 * Further fine tuning of MAX_ANG_VEL may be desired.
 */

@Config
@Autonomous(group = "drive")
public class MaxAngularVeloTuner extends LinearOpMode {
    public static double RUNTIME = 4.0;

    FtcDashboard dash = FtcDashboard.getInstance();
    Canvas field;

    private final int robotRadius = 7;
    private ElapsedTime timer;
    private double maxAngVelocity = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        PersistantTelemetry pt = new PersistantTelemetry(telemetry);

        BaselineMecanumDrive drive = new BaselineMecanumDrive(hardwareMap,pt);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TelemetryPacket packet = new TelemetryPacket();

        telemetry.addLine("Your bot will turn at full speed for " + RUNTIME + " seconds.");
        telemetry.addLine("Please ensure you have enough space cleared.");
        telemetry.addLine("");
        telemetry.addLine("Press start when ready.");
        telemetry.update();

        waitForStart();

        telemetry.clearAll();
        telemetry.update();

        drive.setDrivePower(new Pose2d(0, 0, 1));
        timer = new ElapsedTime();

        while (!isStopRequested() && timer.seconds() < RUNTIME) {
            drive.updatePoseEstimate();

            field = packet.fieldOverlay();

            Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");

            maxAngVelocity = Math.max(poseVelo.getHeading(), maxAngVelocity);

            Pose2d pose = drive.getPoseEstimate();

            double angle = pose.getHeading();

            double arrowX = Math.cos(angle) * robotRadius, arrowY = Math.sin(angle) * robotRadius;
            double x1 = pose.getX() + arrowX  / 2, y1 = pose.getY() + arrowY / 2;
            double x2 = pose.getX() + arrowX, y2 = pose.getY() + arrowY;
            field.strokeLine(x1, y1, x2, y2);

            field.strokeCircle(pose.getX(),pose.getY(),angle);
            dash.sendTelemetryPacket(packet);
        }

        drive.setDrivePower(new Pose2d());

        telemetry.addData("Max Angular Velocity (rad)", maxAngVelocity);
        telemetry.addData("Max Angular Velocity (deg)", Math.toDegrees(maxAngVelocity));
        telemetry.update();

        packet.put("Max Angular Velocity (rad)", maxAngVelocity);

        dash.sendTelemetryPacket(packet);

        while (!isStopRequested()) idle();
    }
}