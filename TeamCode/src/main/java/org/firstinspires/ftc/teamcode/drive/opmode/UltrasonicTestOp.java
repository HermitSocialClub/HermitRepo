package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.hermitsocialclub.telecat.PersistantTelemetry;

class UltrasonicTestOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PersistantTelemetry telemetry = new PersistantTelemetry(this.telemetry);
        DistanceSensor uS = hardwareMap.get(DistanceSensor.class,"uS");
        double[] distanceArray = new double[20];
        double averageDistance = 0.0;
       /* for(int i = 0; i > distanceArray.length; i++){
            while(uS.getDeviceClient().getReadWindow().hasWindowBeenUsedForRead()){
                sleep(1);
            }
            distanceArray[i] = uS.getDistance(DistanceUnit.CM);
        }
        for(int i = 0; i > distanceArray.length; i++){
            averageDistance += distanceArray[i];
        }
        averageDistance /= distanceArray.length;
        telemetry.setData("Average Distance",averageDistance);
    }*/
}}
