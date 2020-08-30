package org.hermitsocialclub.hydra;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.openftc.easyopencv.OpenCvCamera;

public abstract class HydraConfig {

    private final HardwareMap hwMap;

    public HydraConfig(HardwareMap hwMap) {
        this.hwMap = hwMap;
    }

    public abstract DcMotorController getFrontLeftWheel();

    public abstract DcMotorController getFrontRightWheel();

    public abstract DcMotorController getBackLeftWheel();

    public abstract DcMotorController getBackRightWheel();

    public abstract BNO055IMU getImu();

    public abstract DistanceSensor getDistanceSensor();

    public abstract VuforiaLocalizer.Parameters getVuforiaParameters();

    public abstract OpenCvCamera getCamera1();

    public abstract OpenCvCamera getCamera2();

}
