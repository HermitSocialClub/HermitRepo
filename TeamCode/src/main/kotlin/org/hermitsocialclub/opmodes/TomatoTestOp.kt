package org.hermitsocialclub.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.hermitsocialclub.tomato.LibTomato
import org.hermitsocialclub.tomato.TomatoException

@TeleOp
class TomatoTestOp : LinearOpMode() {

    override fun runOpMode() {
        telemetry.addData("Splat", LibTomato.splat())

        var msg = "none"
        try {
            LibTomato.panicTest()
        } catch (e: TomatoException) {
            msg = e.message ?: "null"
        }
        telemetry.addData("Panic?", msg)
        telemetry.update()

        waitForStart()
    }
}
