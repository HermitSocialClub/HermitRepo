package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.telecat.PersistantTelemetry;

@Autonomous (name = "meet 2 forward")
public class Meet2AutoButItFacesForwards extends LinearOpMode {

    //Basic Utility
    BaselineMecanumDrive drive;
    PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);
    DcMotor duck_wheel;
    Servo outtake_arm;

    @Override
    public void runOpMode() throws InterruptedException {

        duck_wheel = hardwareMap.dcMotor.get("duck_wheel");
        outtake_arm = hardwareMap.servo.get("outtake_arm");

        Trajectory toCarousel = drive.trajectoryBuilder(new Pose2d(-40,63,0))
                .splineTo(new Vector2d(-55,60),-20)
                .build();

        Trajectory toDepot = drive.trajectoryBuilder(toCarousel.end())
                .splineTo(new Vector2d(-55,40),90)
                .build();

        Trajectory toWarehouse = drive.trajectoryBuilder(toDepot.end())
                .splineTo(new Vector2d(-15,45),90)
                .splineTo(new Vector2d(10,65),0)
                .splineTo(new Vector2d(45,65),0)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(toCarousel);
        duckForTime(2.5,0.3);
        drive.followTrajectory(toDepot);
        outtake_arm.setPosition(0);
        wait(600);
        outtake_arm.setPosition(0.45);
        drive.followTrajectory(toWarehouse);

    }

    public void duckForTime (double time, double speed){
        double start = System.currentTimeMillis();
        double end = start + time * 1000;

        while (System.currentTimeMillis() < end) {
            duck_wheel.setPower(speed);
        }
    }

   /* public void movedOuttakeArm (double time, double position){
        double start = System.currentTimeMillis();
        double end = start + time * 1000;

        while (System.currentTimeMillis() < end) {
            outtake_arm.setPosition(position);
        }
    }*/
}
