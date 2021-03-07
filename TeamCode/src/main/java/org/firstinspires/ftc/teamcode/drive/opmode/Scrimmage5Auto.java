package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.hermitsocialclub.telecat.PersistantTelemetry;

@Autonomous (name = "Scrimmage5Auto")
public class Scrimmage5Auto extends LinearOpMode {

    PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);


    MotorConfigurationType goBildaOuttake =  MotorConfigurationType.getMotorType(GoBILDA5202Series.class);
    double outTake75Speed;

    @Override
    public void runOpMode() throws InterruptedException {
        BaselineMecanumDrive drive = new BaselineMecanumDrive(hardwareMap,telemetry);

        outTake75Speed = ((.585 * 2 * Math.PI * goBildaOuttake.getMaxRPM() * goBildaOuttake.getAchieveableMaxRPMFraction())/60);

        //drive.wobbleGrab.setPower(1);

        drive.setPoseEstimate(new Pose2d(-63,-15.5,0));

        Trajectory noRings = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(57)
                .build();
        Trajectory strafeRight = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(10)
                .build();
        Trajectory noRingsSpline = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(new Vector2d(0,-36),-60)
                .build();

        telemetry.setDebug("Achievable RPM Fraction",goBildaOuttake.getAchieveableMaxRPMFraction());
        telemetry.setDebug("Achievable Ticks Per Second",goBildaOuttake.getAchieveableMaxTicksPerSecond());
        telemetry.setDebug("hub velocity params",goBildaOuttake.getHubVelocityParams());
        telemetry.setDebug("velocity params",goBildaOuttake.hasExpansionHubVelocityParams());
        telemetry.setDebug("outtakeVelocity",outTake75Speed);


        waitForStart();


        drive.followTrajectory(noRings);

        while(drive.isBusy()){
            telemetry.setDebug("Pose",drive.getPoseEstimate().toString());
        }

        /*drive.followTrajectory(strafeRight);

        while(drive.isBusy()){
            telemetry.setDebug("Pose",drive.getPoseEstimate().toString());
        }*/

        drive.outtake.setVelocity(outTake75Speed, AngleUnit.RADIANS);

        while (opModeIsActive() && Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS)-outTake75Speed) > Math.pow(10,-1)){
            telemetry.setDebug("Outtake Velocity",drive.outtake.getVelocity(AngleUnit.RADIANS));
            telemetry.setDebug("Ticks Outtake Velocity",drive.outtake.getVelocity(AngleUnit.DEGREES));
            telemetry.setDebug("Ticks Outtake Position",drive.outtake.getCurrentPosition());

        }

        int ringsFired = 0;

        while (opModeIsActive() && ringsFired <= 4) {
            telemetry.setDebug("ringsFired",ringsFired);
            drive.kicker.setPower(1);
            sleep(200);
            drive.kicker.setPower(-1);
            sleep(200);
            drive.kicker.setPower(0);
            while (Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS)-outTake75Speed) > Math.pow(10,-1)){
                telemetry.setDebug("Outtake Velocity",drive.outtake.getVelocity(AngleUnit.RADIANS));
            }
            ringsFired++;
        }
    }
}
