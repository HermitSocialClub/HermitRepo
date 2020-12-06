package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogOutput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.network.ControlHubPasswordManager;
import org.firstinspires.ftc.teamcode.util.AnalogUltrasonic;
import org.hermitsocialclub.telecat.PersistantTelemetry;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

@TeleOp(name = "Ultra Test Op")
public class UltrasonicTestOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PersistantTelemetry telemetry = new PersistantTelemetry(this.telemetry);

        AnalogInput echo = hardwareMap.get(AnalogInput.class,"echo");
        AnalogOutput trigger = hardwareMap.get(AnalogOutput.class,"trigger");

        RevBulkData bulkData;

        ExpansionHubEx rev = hardwareMap.get(ExpansionHubEx.class,"Control Hub");

        bulkData = rev.getBulkInputData();

        AnalogUltrasonic ultra = new AnalogUltrasonic(echo,trigger,telemetry,bulkData,rev);

        double[] distanceArray = new double[20];
        double averageDistance = 0.0;

        for (int i = 0; i < distanceArray.length; i++) {
            ultra.pulse();
            while (!ultra.getPulseFinished()){}
            distanceArray[i] = ultra.getDistance();
            telemetry.setData("distance " + (i+1) + ": %f",distanceArray[i]);
        }
        for (int i = 0; i < distanceArray.length; i++) {
            averageDistance += distanceArray[i];
        }
        averageDistance /= distanceArray.length;
        telemetry.setData("average distance: ", averageDistance);
}}
