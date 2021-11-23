package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "DuckTester")
public class DuckTester extends LinearOpMode {

    //
    // double DcMotor = duck_wheel;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor duck_wheel = null;

    @Override
    public void runOpMode () {

        duck_wheel  = hardwareMap.get(DcMotor.class, "duck_wheel");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){

          //  double duck_wheel;
            if (gamepad1.a){
                duck_wheel.setPower(0.7);
            }else {
                duck_wheel.setPower(0);
            }

            if (gamepad1.b){
                duck_wheel.setPower(0.75);
            }else {
                duck_wheel.setPower(0);
            }

            if (gamepad1.x){
                duck_wheel.setPower(0.6);
            }else {
                duck_wheel.setPower(0);
            }

            if (gamepad1.y){
                duck_wheel.setPower(0.65);
            }else {
                duck_wheel.setPower(0);
            }



        }

    }
}
