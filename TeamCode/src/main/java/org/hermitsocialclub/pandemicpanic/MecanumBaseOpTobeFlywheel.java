package org.hermitsocialclub.pandemicpanic;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.hermitsocialclub.telecat.PersistantTelemetry;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TobeFly", group = "Hermit")

public class MecanumBaseOpTobeFlywheel extends LinearOpMode {

    private PersistantTelemetry pt = new PersistantTelemetry(telemetry);
    ElapsedTime runtime = new ElapsedTime();
    private double tobeMaxEncoder = 1;
    private double tobeSpeedThreeEncoder = 0.75;
    private double tobeSpeedTwoEncoder = 0.5;
    private double tobeSpeedOneEncoder = 0.25;
    private double tobeDistanceRatio = 0.0625; //based off observed distance of ring at max rpm
    private double tobePowerRatio;




    //private DistanceSensor sonicHedgehogSensor;
    private DcMotorEx tobeFlywheel;

    private Servo kicker;



    @Override
    public void runOpMode() throws InterruptedException {

        tobeFlywheel = hardwareMap.get(DcMotorEx.class, "tobeFlywheel");
        kicker = hardwareMap.get(Servo.class,"kicker");
        //sonicHedgehogSensor = hardwareMap.get(DistanceSensor.class,"Sonic the Hedgehog");
        //tobePowerRatio = Math.max(sonicHedgehogSensor.getDistance(DistanceUnit.CM) * tobeDistanceRatio,1);


        waitForStart();
        telemetry.speak("Hola. Cómo estás?", "spa", "mx");
        telemetry.update();

        while (opModeIsActive()) {
            //tobePowerRatio = Math.max(sonicHedgehogSensor.getDistance(DistanceUnit.CM) * tobeDistanceRatio,1);


            if (gamepad1.dpad_up){
                tobeFlywheel.setPower(-tobeMaxEncoder);
            }else if (gamepad1.dpad_down){
                tobeFlywheel.setPower(-tobeSpeedOneEncoder);
            }else if (gamepad1.dpad_left){
                tobeFlywheel.setPower(-tobeSpeedTwoEncoder);
            }else if (gamepad1.dpad_right){
                tobeFlywheel.setPower(-tobeSpeedThreeEncoder);
            }
            if (gamepad2.x){
                //tobeFlywheel.setPower(-tobePowerRatio);
            }
            if(gamepad1.right_bumper){
                kicker.setPosition(1);
            }else if(gamepad1.left_bumper) {
                kicker.setPosition(0);
            }
            //pt.setData("raw ultrasonic", sonicHedgehogSensor.getDistance(DistanceUnit.CM));
            //pt.setData("cm", "%.2f cm", sonicHedgehogSensor.getDistance(DistanceUnit.CM));
        }

    }



}
