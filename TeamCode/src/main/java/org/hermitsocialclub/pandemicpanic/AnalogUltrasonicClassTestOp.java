package org.hermitsocialclub.pandemicpanic;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogOutput;

import org.hermitsocialclub.util.AnalogUltrasonic;
import org.hermitsocialclub.telecat.PersistantTelemetry;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
@Disabled
@TeleOp(name = "Ultrasonic Test Op")
public class AnalogUltrasonicClassTestOp extends LinearOpMode {

    AnalogInput echo;
    AnalogOutput trigger;

    AnalogUltrasonic ultra;

    PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        ExpansionHubEx rev = hardwareMap.get(ExpansionHubEx.class,"Control Hub");
        RevBulkData bulkData = rev.getBulkInputData();

        echo = hardwareMap.get(AnalogInput.class,"echo");
        trigger = hardwareMap.get(AnalogOutput.class, "trigger");
        ultra = new AnalogUltrasonic(echo,trigger,telemetry,bulkData,rev);

        waitForStart();

        while (opModeIsActive()){
            ultra.pulse();
            while (!ultra.getPulseFinished()){

            }
            telemetry.setData("Distance to Surface",ultra.getDistance());
        }

    }
}
