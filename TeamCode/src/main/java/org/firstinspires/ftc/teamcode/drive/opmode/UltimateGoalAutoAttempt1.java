package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.SkystoneVuforiaEngine;
import org.hermitsocialclub.telecat.PersistantTelemetry;
import org.opencv.core.Mat;

@Autonomous(name="Ultimate Goal Zone B Attempt 1")
public class UltimateGoalAutoAttempt1 extends LinearOpMode {
    static Trajectory zoneAPath;
    static Trajectory zoneBPath;
    static Trajectory zoneCPath;
    private enum Zone {
        ZONEA(zoneAPath), ZONEB(zoneBPath), ZONEC(zoneCPath);
        Trajectory path;
        Zone(Trajectory path){
            this.path = path;
        }
    }
    Zone dropOffZone;
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();
        PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);
        BaselineMecanumDrive drive = new BaselineMecanumDrive(hardwareMap,telemetry);
        drive.setPoseEstimate(new Pose2d(-63,-50,Math.toRadians(0)));
        dropOffZone = openCVStuff();
        try {
            switch (dropOffZone) {
                case ZONEA: {
                zoneAPath = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineToConstantHeading(new Vector2d(24.00, -60.00), 0)
                        .splineToConstantHeading(new Vector2d(-4, -30), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-4, 0), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-36, -22), Math.toRadians(200))
                        .splineToSplineHeading(new Pose2d(12, -44, 90), 0)
                        .build();
                break;
                }
                case ZONEB: {
                    zoneBPath = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineTo(new Vector2d(-15.00, -50.00), 0)
                            .splineToLinearHeading(new Pose2d(0, 0, 0), Math.toRadians(90))
                            .splineToLinearHeading(new Pose2d(48, -36, 0.00), Math.toRadians(-90))
                            .splineToConstantHeading(new Vector2d(-36, -36), Math.toRadians(90))
                            .splineToConstantHeading(new Vector2d(-20, -36), 0)
                            .splineToConstantHeading(new Vector2d(-40, -26), Math.toRadians(180))
                            .splineToSplineHeading(new Pose2d(20, -36, Math.toRadians(180)), 0)
                            .splineToSplineHeading(new Pose2d(10, -36, Math.toRadians(180)), 0)
                            .build();
                    break;
                }
                case ZONEC: {
                    zoneCPath = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineToConstantHeading(new Vector2d(0, -36), Math.toRadians(90))
                            .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(90))
                            .splineToSplineHeading(new Pose2d(50, -44, Math.toRadians(90)), Math.toRadians(180))
                            .splineToSplineHeading(new Pose2d(-20, -36, Math.toRadians(180)), Math.toRadians(180))
                            .splineToSplineHeading(new Pose2d(-40, -26, 0), Math.toRadians(180))
                            .splineToSplineHeading(new Pose2d(50, -60, Math.toRadians(180)), 0)
                            .build();
                    break;
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        telemetry.setDebug("Localizer",drive.getLocalizer().toString());
        telemetry.setDebug("Wheel Positions",drive.getWheelPositions());
        telemetry.setDebug("Pose Estimate","%.2f, %.2f",drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY());
        telemetry.setData("Acceleration", zoneBPath.acceleration(zoneBPath.duration()).toString());
        telemetry.setData("Duration", zoneBPath.duration());
        telemetry.setData("End", zoneBPath.end().toString());
        telemetry.setData("Start", zoneBPath.start().toString());
        telemetry.setData("Path", zoneBPath.getPath().toString());
        telemetry.setData("Velocity", zoneBPath.velocity(zoneBPath.duration()).toString());
        waitForStart();
        if (isStopRequested()) return;
        runtime.reset();
       /* switch (dropOffZone) {
            case ZONEA: drive.followTrajectory(zoneAPath); break;
            case ZONEB: telemetry.setData("Drivin", "weeee");drive.followTrajectory(zoneBPath); break;
            case ZONEC: drive.followTrajectory(zoneCPath); break;
        }*/
       drive.followTrajectory(zoneBPath);
    }
        private Zone openCVStuff(){
        return Zone.ZONEB;
        }

}
