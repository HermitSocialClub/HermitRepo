package org.hermitsocialclub.localizers

import com.qualcomm.robotcore.hardware.HardwareMap
import com.spartronics4915.lib.T265Localizer
import org.hermitsocialclub.tomato.LibTomato.SLAMRA
import org.hermitsocialclub.tomato.LibTomato.checkBatteryForSlamra

class T265LocalizerPro(hardwareMap: HardwareMap) : T265Localizer(SLAMRA) {
    init {
        // Force an update here even if the camera is not
        // ready yet to prevent NullPointerExceptions
        update()

        // Check battery level because the localizer dies if it doesn't get enough juice
        checkBatteryForSlamra(hardwareMap)
    }
}
