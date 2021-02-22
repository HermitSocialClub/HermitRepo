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

    @Override
    public void runOpMode() throws InterruptedException {
        BaselineMecanumDrive drive = new BaselineMecanumDrive(hardwareMap,telemetry);

        //drive.wobbleGrab.setPower(1);

        drive.setPoseEstimate(new Pose2d(-63,-15.5,0));

        Trajectory noRings = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(57)
                .build();
        Trajectory strafeRight = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(10)
                .build();

        waitForStart();


        drive.followTrajectory(noRings);

        while(drive.isBusy()){
            telemetry.setDebug("Pose",drive.getPoseEstimate().toString());
        }

        /*drive.followTrajectory(strafeRight);

        while(drive.isBusy()){
            telemetry.setDebug("Pose",drive.getPoseEstimate().toString());
        }*/

        drive.outtake.setVelocity(.75 * 2 * Math.PI * goBildaOuttake.getMaxRPM() * goBildaOuttake.getAchieveableMaxRPMFraction(), AngleUnit.RADIANS);

        while (Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS)-(.75 * 2 * Math.PI * goBildaOuttake.getMaxRPM() * goBildaOuttake.getAchieveableMaxRPMFraction())) > Math.pow(10,-4)){
            telemetry.setDebug("Outtake Velocity",drive.outtake.getVelocity());
        }

        int ringsFired = 0;

        while (ringsFired <= 4) {
            telemetry.setDebug("ringsFired",ringsFired);
            drive.kicker.setPower(1);
            sleep(200);
            drive.kicker.setPower(-1);
            sleep(200);
            drive.kicker.setPower(0);
            while (Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS)-(.75 * 2 * Math.PI * goBildaOuttake.getMaxRPM() * goBildaOuttake.getAchieveableMaxRPMFraction())) > Math.pow(10,-4)){
                telemetry.setDebug("Outtake Velocity",drive.outtake.getVelocity());
            }
            ringsFired++;
        }
    }
}
