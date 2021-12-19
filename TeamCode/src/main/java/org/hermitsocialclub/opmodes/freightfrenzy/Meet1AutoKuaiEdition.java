package org.hermitsocialclub.opmodes.freightfrenzy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.spartronics4915.lib.T265Camera;
import com.spartronics4915.lib.T265Helper;
import com.spartronics4915.lib.T265Localizer;
import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.localizers.T265LocalizerPro;
import org.hermitsocialclub.telecat.PersistantTelemetry;

import static org.hermitsocialclub.util.MoveUtils.m;

@Autonomous(name = "Meet1AutoKuaiEdition")
public class Meet1AutoKuaiEdition extends LinearOpMode {

    BaselineMecanumDrive drive;
    PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);
    Pose2d blueStart = new Pose2d(0, 0, m(-90));
    Trajectory toCarouselBlue1;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new BaselineMecanumDrive(hardwareMap, telemetry);
        drive.setLocalizer(new T265LocalizerPro(hardwareMap));

        toCarouselBlue1 = drive.trajectoryBuilder(blueStart, m(-90))
                .back(15.5)
                .build();

        telemetry.setData("Trajectories:", "Initialized");

        waitForStart();

        drive.setPoseEstimate(blueStart);
        drive.followTrajectory(toCarouselBlue1);
    }
}
