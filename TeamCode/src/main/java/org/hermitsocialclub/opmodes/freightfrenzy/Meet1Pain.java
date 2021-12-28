package org.hermitsocialclub.opmodes.freightfrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.telecat.PersistantTelemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "meet1Basic")
public class Meet1Pain extends LinearOpMode {


    //Motors: Left1, Left2, Right2, Right1
    private BaselineMecanumDrive drive;
    private PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);
    private final List<Double> forward =  new ArrayList<>();
    private final List<Double> turnRight = Arrays.asList(-.5,-.5,.5,.5);
    private final List<Double> stop = Arrays.asList(0.0,0.0,.0,.0);
    private final List<Double> back = Arrays.asList(.5,.5,.5,.5);


    public static long goToCarousel = 1000;
    public static long backUp = 350;
    public static long turn = 450;
    public static long crossBarrier = 3000;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new BaselineMecanumDrive(hardwareMap,telemetry);

        for (int i = 0; i < 3; i++) {
            forward.add(-.5);
        }

        waitForStart();

        drive.setRelativeMotorVelocities(forward);
        sleep(goToCarousel);
        drive.setRelativeMotorVelocities(stop);
        drive.outtakeArm.setPosition(.4);
        sleep(200);
        drive.setRelativeMotorVelocities(back);
        sleep(backUp);
        drive.setRelativeMotorVelocities(turnRight);
        sleep(turn);
        drive.setRelativeMotorVelocities(forward);
        sleep(crossBarrier);
        drive.setRelativeMotorVelocities(stop);


    }
}
