package org.hermitsocialclub.opmodes.pandemicpanic;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.telecat.PersistantTelemetry;

@TeleOp(name = "colorTest")
public class testColorSensor extends LinearOpMode {
    private ColorSensor color;
    private final PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);
    //ARGB Block facing grid empty: 503316480
    //
    @Override
    public void runOpMode() throws InterruptedException {
        color = hardwareMap.get(ColorSensor.class, "color");
        waitForStart();
        while (opModeIsActive()) {
            int red = color .red();
            if (red > 80 && red < 100){
                telemetry.setData("has object?: ", "Block");
            }
            else if (red > 190 && red < 220){
                telemetry.setData("has object?: ", "Ball");
            }
            else {
                telemetry.setData("has object?: ", "None");
            }
        }
    }
}