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
    private ColorSensor color_in;
    private final PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);
    //ARGB Block facing grid empty: 503316480
    //
    @Override
    public void runOpMode() throws InterruptedException {
        color = hardwareMap.get(ColorSensor.class, "color");
//        color_in = hardwareMap.get(ColorSensor.class, "color_intake");
        waitForStart();
        while (opModeIsActive()) {
            int red = color.red();
            if (red > 80 && red < 100){
                telemetry.setData("has object?: ", "Block");
            }
            else if (red > 190 && red < 220){
                telemetry.setData("has object?: ", "Ball");
            }
            else {
                telemetry.setData("has object?: ", "None");
            }
            telemetry.setData("Red",color.red());
            telemetry.setData("Blue",color.blue());
            telemetry.setData("Green",color.green());

//            int red_intake = color_in.red();
//            if (red_intake > 80 && red_intake < 100){
//                telemetry.setData("has object_in?: ", "Block");
//            }
//            else if (red_intake > 190 && red_intake < 220){
//                telemetry.setData("has object_in?: ", "Ball");
//            }
//            else {
//                telemetry.setData("has object_in?: ", "None");
//            }
//            telemetry.setData("color_in Red",color_in.red());
//            telemetry.setData("color_in Blue",color_in.blue());
//            telemetry.setData("color_in Green",color_in.green());
        }
    }
}