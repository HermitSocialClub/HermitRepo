package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ArmTest")
public class ArmTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor Arm = hardwareMap.get(DcMotor.class, "Arm");
        waitForStart();

       // gamepad1.left_stick_x =
        Arm.setPower(Math.abs(gamepad1.left_stick_x));

        if (gamepad1.left_stick_x == 0){
            Arm.setPower(0);
        }

    }
}
