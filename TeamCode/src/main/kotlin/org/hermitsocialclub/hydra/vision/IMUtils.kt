package org.hermitsocialclub.hydra.vision

import org.opencv.core.MatOfPoint

/**
 * Port of [imutils](https://github.com/jrosebr1/imutils/) to Java
 * so that we can actually follow all the python OpenCV tutorials
 */
object IMUtils {

    fun grabContours(cnts: List<MatOfPoint>): MatOfPoint {
        var actualCnts: MatOfPoint
        if (cnts.size == 2) {
            actualCnts = cnts[0]
        } else if (cnts.size == 3) {
            actualCnts = cnts[1]
        } else {
            throw RuntimeException(
                "Contours tuple must have length 2 or 3, "
                        + "otherwise OpenCV changed their findContours return "
                        + "signature yet again. Refer to OpenCV's documentation "
                        + "in that case"
            )
        }
        return actualCnts
    }

}