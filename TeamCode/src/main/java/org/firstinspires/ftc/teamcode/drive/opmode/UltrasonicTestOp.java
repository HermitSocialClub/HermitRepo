package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogOutput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.network.ControlHubPasswordManager;
import org.firstinspires.ftc.teamcode.util.AnalogUltrasonic;
import org.hermitsocialclub.telecat.PersistantTelemetry;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

@TeleOp(name = "Ultra Test Op")
public class UltrasonicTestOp extends LinearOpMode {


    //TODO: Check what the resting voltage is
    final double RESTING_VOLTAGE = 4.5;
    final double TRIGGER_PULSE_PERIOD = 10;
    final double MICROSECONDS_TO_INCHES = 148.0;

    RevBulkData bulk;

    ExpansionHubEx rev;

    private ElapsedTime time = new ElapsedTime();
    private ElapsedTime echoTime = new ElapsedTime();
    private ElapsedTime triggerTime = new ElapsedTime();

    private FtcDashboard dash;

    PersistantTelemetry telemetry = new PersistantTelemetry(super.telemetry);
    double distance;

    private boolean pulseFinished = false;

    DigitalChannel trigger;
    AnalogInput echo;


    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();
        dash.setTelemetryTransmissionInterval(25);

        echo = hardwareMap.get(AnalogInput.class,"echo");

        trigger = hardwareMap.get(DigitalChannel.class,"trigger");

        trigger.setMode(DigitalChannel.Mode.OUTPUT);

        trigger.setState(false);

        //AnalogInput echo = hardwareMap.get(AnalogInput.class,"echo");
        //AnalogOutput trigger = hardwareMap.get(AnalogOutput.class,"trigger");

        RevBulkData bulkData;

        ExpansionHubEx rev = hardwareMap.get(ExpansionHubEx.class,"Control Hub");

        bulkData = rev.getBulkInputData();

        //AnalogUltrasonic ultra = new AnalogUltrasonic(echo,trigger,telemetry,bulkData,rev);

        double[] distanceArray = new double[20];
        double averageDistance = 0.0;

        waitForStart();

        while (opModeIsActive()){
            trigger.setState(true);
            telemetry.setDebug("trigger",trigger.getState());
            telemetry.setDebug("echo",echo.getVoltage());
            triggerTime.reset();
            while(triggerTime.nanoseconds() / 1000 <= TRIGGER_PULSE_PERIOD+5 && opModeIsActive()){ }
            trigger.setState(false);
            echoTime.reset();
            while (echo.getVoltage()<RESTING_VOLTAGE && opModeIsActive()){
                TelemetryPacket pack = new TelemetryPacket();
                telemetry.setDebug("in loop",echoTime);
                pack.put("in loop",echoTime);
                telemetry.setDebug("echo",echo.getVoltage());
                pack.put("echo",echo.getVoltage());
                dash.sendTelemetryPacket(pack);
                telemetry.setDebug("trigger",trigger.getState());
            }
        /*for (int i = 0; i < distanceArray.length; i++) {
            echo.getState();
            while (!pulseFinished){}
            distanceArray[i] = distance;
            telemetry.setData("distance " + (i+1) + ": %f",distanceArray[i]);
        }
        for (int i = 0; i < distanceArray.length; i++) {
            averageDistance += distanceArray[i];
        }
        averageDistance /= distanceArray.length;
        telemetry.setData("average distance: ", averageDistance);*/
        }
}
    public void pulse(){
        pulseFinished = false;
        trigger.setState(true);
        triggerTime.reset();
        while(triggerTime.nanoseconds() * 1000 <= TRIGGER_PULSE_PERIOD){ }
        trigger.setState(false);
        echoTime.reset();
        bulk = rev.getBulkInputData();
        while (bulk.getAnalogInputValue(echo) > RESTING_VOLTAGE){
            bulk = rev.getBulkInputData();
            telemetry.setDebug("INPUT QUERY DATA",bulk.getAnalogInputValue(echo));
    }
        distance = (echoTime.nanoseconds()*1000)/MICROSECONDS_TO_INCHES;
        pulseFinished = true;
    }
}
