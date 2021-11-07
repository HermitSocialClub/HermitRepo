package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Meet3Bot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AssetsTrajectoryManager;
import org.firstinspires.ftc.teamcode.vision.SkystoneVuforiaEngine;
import org.hermitsocialclub.telecat.PersistantTelemetry;

import java.io.IOException;

@Autonomous(name = "Long Path Test")
public class LongPathTest extends LinearOpMode {
    Trajectory trajectory;

    @Override
    public void runOpMode() throws InterruptedException {

        PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);
        SkystoneVuforiaEngine vuforiaEngine = SkystoneVuforiaEngine.get(telemetry);
        telemetry.setData("vuforia intitialized?",vuforiaEngine.hasAlreadyBeenInit);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap,vuforiaEngine);
        TrajectoryGroupConfig tgc =
                new TrajectoryGroupConfig
                        (drive.constraints.maxVel,drive.constraints.maxAccel,
                                drive.constraints.maxAngVel,
                                drive.constraints.maxAngAccel,
                                15,10.75,
                                TrajectoryGroupConfig.DriveType.MECANUM,
                                Meet3Bot.TRACK_WIDTH,
                                Meet3Bot.TRACK_WIDTH,1.0);
        try {
            trajectory = AssetsTrajectoryManager.loadConfig("PewPewTest").
                    toTrajectory(tgc);
        }catch (IOException e){
            e.printStackTrace();
        }
        telemetry.setData("End",trajectory.end().toString());
        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);
    }
}
