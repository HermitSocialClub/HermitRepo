package org.hermitsocialclub.hydra

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.openftc.easyopencv.OpenCvCamera

abstract class HydraConfig(val hw: HardwareMap) {

    abstract val frontLeftWheel: DcMotorController
    abstract val frontRightWheel: DcMotorController
    abstract val backLeftWheel: DcMotorController
    abstract val backRightWheel: DcMotorController
    abstract val imu: BNO055IMU
    abstract val distanceSensor: DistanceSensor
    abstract var vuforiaParameters: VuforiaLocalizer.Parameters
    abstract val camera1: OpenCvCamera
    abstract val camera2: OpenCvCamera
}
