package org.hermitsocialclub.pandemicpanic;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static org.hermitsocialclub.util.TuningController.goBildaOuttake;
import static org.hermitsocialclub.pandemicpanic.Ver3MecanumBaseOp2021.SPEED_PERCENT;
@Disabled
@TeleOp(name = "Outtake Tester Op")
public class OuttakeTestOp extends LinearOpMode {

    double outTake75Speed = -((SPEED_PERCENT * 2 * Math.PI * goBildaOuttake.getMaxRPM() * goBildaOuttake.getAchieveableMaxRPMFraction()) / 60);
    DcMotorEx outtake;

    @Override
    public void runOpMode() throws InterruptedException {
        outtake = hardwareMap.get(DcMotorEx.class,"takeruFlyOut");
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.right_trigger > .03) {
                outtake.setVelocity(outTake75Speed, AngleUnit.RADIANS);
            }
        }
    }
}
