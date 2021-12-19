package org.hermitsocialclub.localizers

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.spartronics4915.lib.T265Camera
import com.spartronics4915.lib.T265Helper
import com.spartronics4915.lib.T265Localizer
import org.hermitsocialclub.drive.config.DriveConstants

class T265LocalizerPro(hardwareMap: HardwareMap) : T265Localizer(
    T265Helper.getCamera(
        T265Camera.OdometryInfo(Pose2d(DriveConstants.slamraX, DriveConstants.slamraY), 0.0),
        hardwareMap.appContext
    ).also {
        if(!it.isStarted) {
            it.start()
        }
    }
)
