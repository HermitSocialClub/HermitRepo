package org.hermitsocialclub.hydra.vision

import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f

object VisionUtils {

    fun toMatOfPoint2f(input: MatOfPoint): MatOfPoint2f {
        return MatOfPoint2f(*input.toArray())
    }

}