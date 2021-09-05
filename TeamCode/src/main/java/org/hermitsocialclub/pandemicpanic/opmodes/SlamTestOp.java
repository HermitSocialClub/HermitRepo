package org.hermitsocialclub.pandemicpanic.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import org.hermitsocialclub.telecat.PersistantTelemetry;

import static android.os.SystemClock.sleep;

@TeleOp (name = "SlamTestOp")
public class SlamTestOp extends OpMode {

    Pose2d startingPose = new Pose2d(0, 0, new Rotation2d());

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private static T265Camera slamra;

    PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);


    @Override
    public void init() {
        if(slamra == null) {
            slamra = new T265Camera(new Transform2d(), .8, hardwareMap.appContext);
        }
        slamra.setPose(startingPose);

    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        slamra.stop();
        slamra.start();
        telemetry.setDebug("started","slamra");
    }

    @Override
    public void loop() {
        final int robotRadius = 9; // inches

        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();
        T265Camera.CameraUpdate update;

        update = slamra.getLastReceivedCameraUpdate();
        if(update == null){
            telemetry.setDebug("slamra update", "null");
            packet.addLine("Slamra Update Null");
            sleep(5000);
            return;
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
    }

    @Override
    public void stop() {
        slamra.stop();
        while (slamra.isStarted()){}
    }




}
