package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.telecat.PersistantTelemetry;

@TeleOp (name = "DuckTester")
public class DuckTester extends LinearOpMode {

    //
    // double DcMotor = duck_wheel;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor duck_wheel = null;
    private BaselineMecanumDrive drive;
    private PersistantTelemetry telemetry;
    private int startingPosition;


    @Override
    public void runOpMode () {
        telemetry = new PersistantTelemetry(super.telemetry);
        drive = new BaselineMecanumDrive(hardwareMap, telemetry);
        startingPosition = drive.duck_wheel.getCurrentPosition();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){

//          //  double duck_wheel;
            if (gamepad1.x){
//                duck_wheel.setPower(0.7);
                telemetry.setData("eeee", startingPosition);
                drive.duck_wheel.setTargetPosition(startingPosition + (int) (2 * drive.duck_wheel.getMotorType().getTicksPerRev()));
                drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.duck_wheel.setPower(-0.75);
            }
            if (gamepad1.y){
                startingPosition = drive.duck_wheel.getCurrentPosition();
                drive.duck_wheel.setPower(0);
            }
//
//            if (gamepad1.b){
//                duck_wheel.setPower(0.75);
//            }else {
//                duck_wheel.setPower(0);
//            }
//
//            if (gamepad1.x){
//                duck_wheel.setPower(0.6);
//            }else {
//                duck_wheel.setPower(0);
//            }


//            if (gamepad1.y){
//                duck_wheel.setPower(0.65);
//            }else {
//                duck_wheel.setPower(0);
//            }


        }

    }
}
