package org.hermitsocialclub.pandemicpanic;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.hermitsocialclub.telecat.PersistantTelemetry;
@Disabled
@TeleOp(name = "ColorTestOp")
public class ColorTestOp extends LinearOpMode {
    private RevColorSensorV3 color;
    private PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);
    @Override
    public void runOpMode() throws InterruptedException {
        color = hardwareMap.get(RevColorSensorV3.class,"color");
        waitForStart();
        while (opModeIsActive()){
            telemetry.setDebug("LIGHT_SENSED",color.getLightDetected());
            telemetry.setDebug("DISTANCE",color.getDistance(DistanceUnit.INCH));
            telemetry.setDebug("RAW_LIGHT",color.getRawLightDetected());
            telemetry.setDebug("RAW_OPTICAL",color.rawOptical());
        }
    }
}
