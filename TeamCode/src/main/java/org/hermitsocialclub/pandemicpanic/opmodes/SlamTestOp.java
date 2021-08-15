package org.hermitsocialclub.pandemicpanic.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import org.hermitsocialclub.telecat.PersistantTelemetry;

@TeleOp (name = "SlamTestOp")
public class SlamTestOp extends LinearOpMode {

    Pose2d startingPose = new Pose2d(0, 0, new Rotation2d());

    Transform2d cameraToRobot = new Transform2d();

    double encoderMeasurementCovariance = 1;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    T265Camera slamra;
    T265Camera.CameraUpdate update;

    PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);

    private static T265Camera slamJam;

    @Override
    public void runOpMode() throws InterruptedException {
        if(slamra == null) {
            slamra = new T265Camera(cameraToRobot, encoderMeasurementCovariance, hardwareMap.appContext);
        }
        slamra.setPose(startingPose);

        slamra.start();
        telemetry.setDebug("started","slamra");

        waitForStart();

        while (opModeIsActive()) {
            final int robotRadius = 9; // inches

            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();

            update = slamra.getLastReceivedCameraUpdate();
            if(update == null){
                telemetry.setDebug("slamra update", "null");
                packet.addLine("Slamra Update Null");
                sleep(5000);
                break;
            }
            Translation2d translation = new Translation2d(update.pose.getTranslation().getX() / 0.0254, update.pose.getTranslation().getY() / 0.0254);
            Rotation2d rotation = update.pose.getRotation();

            field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
            double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
            double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
            double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
            field.strokeLine(x1, y1, x2, y2);
            packet.put("Pose", update.pose.toString());

            dashboard.sendTelemetryPacket(packet);

            telemetry.setDebug("Pose",update.pose.toString());
            if(isStopRequested()){
                slamra.stop();
            }

        }
        slamra.stop();
    }
}
