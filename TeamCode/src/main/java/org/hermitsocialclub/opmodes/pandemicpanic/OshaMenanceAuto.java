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

@Autonomous(name = "OshaMenanceAutoAttempt", group = "Hermit")
class OshaMenanceAuto extends LinearOpMode {

    private PersistantTelemetry pt = new PersistantTelemetry(telemetry);
    //PushBotConfiguration drive = new PushBotConfiguration();
    ElapsedTime runtime = new ElapsedTime();
    BaselineMecanumDrive duck = new BaselineMecanumDrive(hardwareMap, pt);
    DcMotor intakeOuttake;
   // Servo claw;
   // DcMotor duck_wheel;


    @Override
    public void runOpMode() throws InterruptedException {

        intakeOuttake = hardwareMap.dcMotor.get("intakeOuttake");
      //  claw = hardwareMap.servo.get("claw");
       // duck_wheel = hardwareMap.dcMotor.get("duck_wheel");

        Trajectory m1 = duck.trajectoryBuilder(new Pose2d(-13.00, 62.00, -90.00))
                .splineTo(new Vector2d(-13, 40), -90)
                /*.addDisplacementMarker(() -> {

                })*/
                .build();


        Trajectory m2 = duck.trajectoryBuilder(m1.end())
                .splineTo(new Vector2d(-15, -40), -90)
                //.addDisplacementMarker()
                /*.addDisplacementMarker(() -> {

                })*/
                .build();

        Trajectory m3 = duck.trajectoryBuilder(m2.end())
                .splineTo(new Vector2d(10, -62), 0)
                .splineTo(new Vector2d(50, -62), 0)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        duck.followTrajectory(m1);
        moveIntakeOuttake(0,1);
       // moveDuckWheel(0.7, 6);
        duck.followTrajectory(m2);
        //moveClaw(0.3,1);
       // claw.setPosition(0);
        duck.followTrajectory(m3);


    }

    public void moveIntakeOuttake(double speed, double time) {

        double start = System.currentTimeMillis();
        double end = start + time * 1000;
        while (System.currentTimeMillis() < end) {
            intakeOuttake.setPower(speed);
        }

    }

   /* public void moveDuckWheel(double speed, double time) {

        double start = System.currentTimeMillis();
        double end = start + time * 1000;
        while (System.currentTimeMillis() < end) {
            duck_wheel.setPower(speed);
        }
    }*/

   /* public void moveClaw(double position, double time) {

        double start = System.currentTimeMillis();
        double end = start + time * 1000;
        while (System.currentTimeMillis() < end) {
            claw.setPosition(position);
        }

    }*/
}