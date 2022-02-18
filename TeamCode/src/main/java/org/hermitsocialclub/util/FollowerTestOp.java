package org.hermitsocialclub.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.telecat.PersistantTelemetry;

@TeleOp(name = "Wall Follower Test")
public class FollowerTestOp extends OpMode {

    BaselineMecanumDrive drive;
    PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);

    @Override
    public void init() {

        drive = new BaselineMecanumDrive(hardwareMap,telemetry);

    }

    @Override
    public void loop() {

        telemetry.setData("Follower Velocity Right", drive.intake.getVelocity());
        telemetry.setData("Follower Velocity Left", drive.leftFollower.getVelocity());

    }
}
