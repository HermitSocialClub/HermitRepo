package org.hermitsocialclub.opmodes.pandemicpanic;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.hermitsocialclub.telecat.PersistantTelemetry;

@TeleOp(name = "ColorTestOp")
public class ColorTestOp extends LinearOpMode {
    private ColorSensor color;
    private final PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        color = hardwareMap.get(ColorSensor.class, "color");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.setDebug("ALPHA", color.alpha());
            telemetry.setDebug("ARGB", color.argb());
            telemetry.setDebug("GREEN", color.green());
            telemetry.setDebug("RED", color.red());
        }
    }
}
