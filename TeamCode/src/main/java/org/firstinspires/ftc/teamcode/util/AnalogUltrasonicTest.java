package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogOutput;

public class AnalogUltrasonicTest extends LinearOpMode {
    AnalogInput ultrasonic;
    AnalogOutput ultrasonicPing;

    @Override
    public void runOpMode() throws InterruptedException {
        ultrasonic = hardwareMap.get(AnalogInput.class,"ultra");
        ultrasonicPing =  hardwareMap.get(AnalogOutput.class,"ultra");
        ultrasonicPing.setAnalogOutputFrequency(60);
    }
}
