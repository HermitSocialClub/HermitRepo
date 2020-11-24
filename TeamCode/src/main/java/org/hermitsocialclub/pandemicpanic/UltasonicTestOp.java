package org.hermitsocialclub.pandemicpanic;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogOutput;

import org.firstinspires.ftc.teamcode.util.AnalogUltrasonic;
import org.hermitsocialclub.telecat.PersistantTelemetry;

@TeleOp(name = "Ultrasonic Test Op")
public class UltasonicTestOp extends LinearOpMode {

    AnalogInput echo;
    AnalogOutput trigger;

    AnalogUltrasonic ultra;

    PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);

    @Override
    public void runOpMode() throws InterruptedException {

        echo = hardwareMap.get(AnalogInput.class,"echo");
        trigger = hardwareMap.get(AnalogOutput.class, "trigger");
        ultra = new AnalogUltrasonic(echo,trigger,telemetry);

        waitForStart();

        while (opModeIsActive()){
            ultra.pulse();
            while (!ultra.getPulseFinished()){

            }
            telemetry.setData("Distance to Surface",ultra.getDistance());
        }

    }
}
