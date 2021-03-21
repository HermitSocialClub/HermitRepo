package org.hermitsocialclub.pandemicpanic;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.opmode.BaselineMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;
import org.hermitsocialclub.hydra.vision.StaccDetecc;
import org.hermitsocialclub.hydra.vision.VisionPipeline;
import org.hermitsocialclub.hydra.vision.VisionSemaphore;
import org.hermitsocialclub.telecat.PersistantTelemetry;

@Autonomous(name = "Scrimmage5Auto")
public class Scrimmage5Auto extends LinearOpMode {

    private static final double SPEED_PERCENT = 0.625;
    private PersistantTelemetry telemetry;
    private MotorConfigurationType goBildaOutTake;
    private BaselineMecanumDrive drive;
    private double outTake75Speed;
    private int ringStack = 0;
    private StaccDetecc stackDetector;
    private VisionSemaphore semaphore;
    private VisionPipeline visionPipeline;
    private ElapsedTime time;

    @Override
    public void runOpMode() throws InterruptedException {
        // Init stuff
        this.telemetry = new PersistantTelemetry(super.telemetry);
        this.goBildaOutTake = MotorConfigurationType.getMotorType(GoBILDA5202Series.class);
        this.outTake75Speed = ((SPEED_PERCENT * 2 * Math.PI * goBildaOutTake.getMaxRPM() * goBildaOutTake.getAchieveableMaxRPMFraction()) / 60);
        this.drive = new BaselineMecanumDrive(hardwareMap, telemetry);
        this.drive.setPoseEstimate(new Pose2d(-63, -15.5, 0));

        this.stackDetector = new StaccDetecc();
        this.semaphore = new VisionSemaphore();
        this.visionPipeline = new VisionPipeline(hardwareMap, telemetry, stackDetector, semaphore);
        this.time = new ElapsedTime();
        this.time.reset();

        // Find which Zone the ring stack is in
        while (time.seconds() < 4 && !isStarted() && !isStopRequested()) {
            ringStack = getZoneFromOpenCV();
            telemetry.setDebug("detected", ringStack);
        }
        telemetry.setDebug("done", "viewing");
        telemetry.setDebug("final position", ringStack);

        // Build trajectories
        Trajectory strafeRight = drive
                .trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(10)
                .build();
        Trajectory launchSpline = drive
                .trajectoryBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(() -> drive.wobbleGrab.setPower(1))
                .splineToConstantHeading(new Vector2d(-3, -42), 60)
                .build();
        Trajectory oneSpline = drive
                .trajectoryBuilder(launchSpline.end())
                .forward(12)
                .build();
        Trajectory fourSpline = drive
                .trajectoryBuilder(launchSpline.end())
                .splineToConstantHeading(new Vector2d(33, -68), 0)
                .build();
        Trajectory fourBack = drive
                .trajectoryBuilder(fourSpline.end())
                .back(30)
                .build();
        Trajectory noSquare = drive
                .trajectoryBuilder(launchSpline.end().plus(new Pose2d(0, 0, Math.toRadians(-60))))
                .forward(5.5)
                .build();
        Trajectory noRingDisengage = drive
                .trajectoryBuilder(noSquare.end())
                .back(3)
                .build();
        Trajectory noRings = drive
                .trajectoryBuilder(noRingDisengage.end().plus(new Pose2d(0, 0, Math.toRadians(-120))))
                .splineToConstantHeading(new Vector2d(-36.50, -46.50), 180)
                .addDisplacementMarker(() -> drive.wobbleGrab.setPower(1))
                .build();
        Trajectory noRingsDrop = drive
                .trajectoryBuilder(noRings.end(), 180)
                .splineToLinearHeading(new Pose2d(-10.50, -54.50, 0), 0)
                .build();

        telemetry.setDebug("Achievable RPM Fraction", goBildaOutTake.getAchieveableMaxRPMFraction());
        telemetry.setDebug("Achievable Ticks Per Second", goBildaOutTake.getAchieveableMaxTicksPerSecond());
        telemetry.setDebug("hub velocity params", goBildaOutTake.getHubVelocityParams());
        telemetry.setDebug("velocity params", goBildaOutTake.hasExpansionHubVelocityParams());
        telemetry.setDebug("outtakeVelocity", outTake75Speed);

        waitForStart();

        // Have the robot drive out
        drive.followTrajectory(launchSpline);

        while (drive.isBusy()) {
            telemetry.setDebug("Pose", drive.getPoseEstimate().toString());
        }

        // Start the outtake and wait for it to reach full speed
        drive.outtake.setVelocity(outTake75Speed, AngleUnit.RADIANS);
        while (opModeIsActive() && Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) - outTake75Speed) > Math.pow(10, -1)) {
            telemetry.setDebug("Outtake Velocity", drive.outtake.getVelocity(AngleUnit.RADIANS));
            telemetry.setDebug("Ticks Outtake Velocity", drive.outtake.getVelocity(AngleUnit.DEGREES));
            telemetry.setDebug("Ticks Outtake Position", drive.outtake.getCurrentPosition());
        }

        // Fire all the rings!
        int ringsFired = 0;
        while (opModeIsActive() && ringsFired <= 5) {
            telemetry.setDebug("ringsFired", ringsFired);
            drive.kicker.setPower(1);
            sleep(200);
            drive.kicker.setPower(-1);
            sleep(200);
            drive.kicker.setPower(0);
            while (Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS) - outTake75Speed) > Math.pow(10, -1)) {
                telemetry.setDebug("Outtake Velocity", drive.outtake.getVelocity(AngleUnit.RADIANS));
            }
            ringsFired++;
        }

        // Reset outtake
        drive.outtake.setVelocity(0, AngleUnit.RADIANS);
        while (opModeIsActive() && Math.abs(drive.outtake.getVelocity(AngleUnit.RADIANS)) > Math.pow(10, -1)) {
            telemetry.setDebug("Outtake Velocity", drive.outtake.getVelocity(AngleUnit.RADIANS));
            telemetry.setDebug("Ticks Outtake Velocity", drive.outtake.getVelocity(AngleUnit.DEGREES));
            telemetry.setDebug("Ticks Outtake Position", drive.outtake.getCurrentPosition());
        }

        // Move wobble goal to target zone
        if (ringStack == 0) {
            drive.turn(Math.toRadians(-60));
            drive.followTrajectory(noSquare);
        } else if (ringStack == 1) {
            drive.followTrajectory(oneSpline);
        } else if (ringStack == 4) {
            drive.followTrajectory(fourSpline);
        }
        drive.liftWobble(-140, 0.45, AngleUnit.DEGREES, 1500);
        drive.wobbleGrab.setPower(-1);
        sleep(300);
        drive.wobbleGrab.setPower(0);
        sleep(200);
        if (ringStack == 0) {
            drive.followTrajectory(noRingDisengage);
        }
        if (ringStack == 4) {
            drive.followTrajectory(fourBack);
        } else if (ringStack == 0) {
            drive.turn(Math.toRadians(-120));
            waitForDrive();
            drive.followTrajectory(noRings);
            sleep(300);
            drive.liftWobble(30, 0.45, AngleUnit.DEGREES, 1500);
            drive.followTrajectory(noRingsDrop);
            waitForDrive();
        }
        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    private void waitForDrive() {
        // noinspection StatementWithEmptyBody
        while (this.drive.isBusy()) {}
    }

    private int getZoneFromOpenCV() {
        semaphore.waitForFrame();
        return stackDetector.getLastStackHeight();
    }

}
