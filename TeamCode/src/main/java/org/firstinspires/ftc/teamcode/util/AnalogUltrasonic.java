package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogOutput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.hermitsocialclub.telecat.PersistantTelemetry;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.util.concurrent.TimeUnit;

public class AnalogUltrasonic {

    //TODO: Check what the resting voltage is
    final double RESTING_VOLTAGE = 4.5;
    final double TRIGGER_PULSE_PERIOD = 10;
    final double MICROSECONDS_TO_INCHES = 148.0;

    AnalogInput echo;
    AnalogOutput trigger;

    RevBulkData bulk;

    ExpansionHubEx rev;

    private ElapsedTime time = new ElapsedTime();
    private ElapsedTime echoTime = new ElapsedTime();
    private ElapsedTime triggerTime = new ElapsedTime();

    PersistantTelemetry telemetry;
    double distance;

    private boolean pulseFinished = false;

    public AnalogUltrasonic(AnalogInput echo, AnalogOutput trigger, PersistantTelemetry telemetry, RevBulkData bulk,
                            ExpansionHubEx rev){
        this.echo = echo;
        this.trigger = trigger;

        this.bulk = bulk;

        this.rev = rev;

        this.telemetry = telemetry;

        trigger.setAnalogOutputMode((byte) 1);
    }

    public void pulse(){
        pulseFinished = false;
        triggerTime.reset();
        trigger.setAnalogOutputVoltage(640);
        while(triggerTime.nanoseconds() * 1000 <= TRIGGER_PULSE_PERIOD){}
        telemetry.setData("trigger time %f microseconds",triggerTime.nanoseconds()*1000);
        trigger.setAnalogOutputVoltage(0);
        echoTime.reset();
        bulk = rev.getBulkInputData();
        while (bulk.getAnalogInputValue(echo) > RESTING_VOLTAGE){bulk = rev.getBulkInputData();
        telemetry.setData("Bulk Analog Input",bulk.getAnalogInputValue(echo));
        telemetry.setData("echo time %f microseconds",echoTime.nanoseconds()*1000);}
        distance = (echoTime.nanoseconds()*1000)/MICROSECONDS_TO_INCHES;
        pulseFinished = true;
    }

    public double getDistance(){
        return distance;
    }

    public long getPulseUptime(TimeUnit unit){
        return echoTime.time(unit);
    }

    public boolean getPulseFinished(){
        return pulseFinished;
    }

}
