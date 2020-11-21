package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(name = "HC-SR04 Ultrasonic Sensor", description = "Speedy Ultrasonic Sensor from Adafruit", xmlTag = "HCSR04")
class SensorUltraSonicHedgehogButBetter extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    final I2cAddr ADDRESS_I2C_DEFAULT = new I2cAddr(0x57);
    @Override
    public Manufacturer getManufacturer()
    {

        return Manufacturer.Adafruit;
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    @Override
    public String getDeviceName()
    {

        return "Adafruit MCP9808 Temperature Sensor";
    }

    public SensorUltraSonicHedgehogButBetter(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);


        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }
}
