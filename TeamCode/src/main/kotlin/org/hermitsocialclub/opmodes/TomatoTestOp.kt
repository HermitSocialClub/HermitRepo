package org.hermitsocialclub.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.hermitsocialclub.tomato.LibTomato

@TeleOp
class TomatoTestOp : LinearOpMode() {

    override fun runOpMode() {
        telemetry.addData("Splat", LibTomato.splat())
        waitForStart()
    }

}
