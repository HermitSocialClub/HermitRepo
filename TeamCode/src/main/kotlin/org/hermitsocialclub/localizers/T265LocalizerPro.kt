package org.hermitsocialclub.localizers

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.spartronics4915.lib.T265Camera
import com.spartronics4915.lib.T265Helper
import com.spartronics4915.lib.T265Localizer
import org.hermitsocialclub.drive.config.DriveConstants
import org.hermitsocialclub.util.Jukebox

class T265LocalizerPro(hardwareMap: HardwareMap) : T265Localizer(
    T265Helper.getCamera(
        T265Camera.OdometryInfo(Pose2d(DriveConstants.slamraX, DriveConstants.slamraY), 0.0),
        hardwareMap.appContext
    ).also {
        if(!it.isStarted) {
            it.start()
        }
    }
) {
    init {
        // Force an update here even if the camera is not
        // ready yet to prevent NullPointerExceptions
        update()

        // Check battery level because the localizer dies if it doesn't get enough juice
        hardwareMap.voltageSensor.entrySet().stream()
            .mapToDouble { it.value.voltage }
            .filter { it > 0 }
            .min()
            .ifPresent { batteryLevel ->
                if(batteryLevel < 12) {
                    Jukebox.setTelemetryWarning(String.format(
                        "T265LocalizerPro will probably not run correctly (battery voltage %.2fV < 12V)!",
                        batteryLevel
                    ))
                }
            }
    }
}
