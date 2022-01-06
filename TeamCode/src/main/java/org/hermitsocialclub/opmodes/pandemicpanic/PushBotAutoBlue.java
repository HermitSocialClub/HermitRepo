package org.hermitsocialclub.opmodes.pandemicpanic;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.telecat.PersistantTelemetry;

@Autonomous(name = "PushBotAutonomousBlue", group = "Hermit")
public
class PushBotAutoBlue extends LinearOpMode {

    private PersistantTelemetry pt = new PersistantTelemetry(telemetry);
    //PushBotConfiguration drive = new PushBotConfiguration();
    ElapsedTime runtime = new ElapsedTime();
    BaselineMecanumDrive duck = new BaselineMecanumDrive(hardwareMap, pt);
    DcMotor arm;
    Servo claw;
    DcMotor duck_wheel;


    @Override
    public void runOpMode() throws InterruptedException {

        arm = hardwareMap.dcMotor.get("arm");
        claw = hardwareMap.servo.get("claw");
        duck_wheel = hardwareMap.dcMotor.get("duck_wheel");

        Trajectory m1 = duck.trajectoryBuilder(new Pose2d(-40.00, 62.00, 0.00))
                .splineTo(new Vector2d(-55, 62), 0)
                /*.addDisplacementMarker(() -> {

                })*/
                .build();


        Trajectory m2 = duck.trajectoryBuilder(m1.end())
                .addDisplacementMarker(() -> {

                    moveArm(0.3, 4);

                })
                .splineTo(new Vector2d(-15, 40), -75)
                //.addDisplacementMarker()
                /*.addDisplacementMarker(() -> {

                })*/
                .build();

        Trajectory m3 = duck.trajectoryBuilder(m2.end())
                .splineTo(new Vector2d(10, 62), 0)
                .splineTo(new Vector2d(50, 62), 0)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        duck.followTrajectory(m1);
        moveDuckWheel(0.7, 6);
        duck.followTrajectory(m2);
        //moveClaw(0.3,1);
        claw.setPosition(0);
        duck.followTrajectory(m3);


    }

    public void moveArm(double speed, double time) {

        double start = System.currentTimeMillis();
        double end = start + time * 1000;
        while (System.currentTimeMillis() < end) {
            arm.setPower(speed);
        }

    }

    public void moveDuckWheel(double speed, double time) {

        double start = System.currentTimeMillis();
        double end = start + time * 1000;
        while (System.currentTimeMillis() < end) {
            duck_wheel.setPower(speed);
        }
    }

    public void moveClaw(double position, double time) {

        double start = System.currentTimeMillis();
        double end = start + time * 1000;
        while (System.currentTimeMillis() < end) {
            claw.setPosition(position);
        }

    }
}