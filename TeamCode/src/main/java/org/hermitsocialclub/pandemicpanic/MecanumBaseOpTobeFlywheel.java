package org.hermitsocialclub.pandemicpanic;

import com.qualcomm.hardware.motors.GoBILDA5201Series;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.hermitsocialclub.telecat.PersistantTelemetry;
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TobeFly", group = "Hermit")

public class MecanumBaseOpTobeFlywheel extends LinearOpMode {

    private PersistantTelemetry pt = new PersistantTelemetry(telemetry);
    ElapsedTime runtime = new ElapsedTime();
    private double ticksPerRevolution = MotorConfigurationType.getMotorType(GoBILDA5201Series.class).getTicksPerRev();
    private double tobeMaxEncoder = MotorConfigurationType.getMotorType(GoBILDA5201Series.class).getAchieveableMaxTicksPerSecond();;
    private double tobeSpeedThreeEncoder = tobeMaxEncoder * .5;
    private double tobeSpeedTwoEncoder = 0.6 * tobeMaxEncoder ;
    private double tobeSpeedOneEncoder = 0.25 * tobeMaxEncoder;
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
                tobeFlywheel.setPower(1);
                //tobeFlywheel.setVelocity(2 * Math.PI * -tobeMaxEncoder / ticksPerRevolution, AngleUnit.RADIANS);
            }else if (gamepad1.dpad_down){
                tobeFlywheel.setPower(.25);
                //tobeFlywheel.setVelocity(2 * Math.PI * -tobeSpeedOneEncoder / ticksPerRevolution, AngleUnit.RADIANS);
            }else if (gamepad1.dpad_left){
                tobeFlywheel.setPower(.62);
                //tobeFlywheel.setVelocity(2 * Math.PI * -tobeSpeedTwoEncoder / ticksPerRevolution, AngleUnit.RADIANS);
            }else if (gamepad1.dpad_right){
                tobeFlywheel.setPower(.75);
                //tobeFlywheel.setVelocity(2 * Math.PI * -tobeSpeedThreeEncoder / ticksPerRevolution, AngleUnit.RADIANS);
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
