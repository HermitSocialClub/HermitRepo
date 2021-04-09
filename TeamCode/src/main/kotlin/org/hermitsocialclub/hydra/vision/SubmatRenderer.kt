package org.hermitsocialclub.hydra.vision

import org.opencv.core.Mat
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc

class SubmatRenderer(var submat: Rect) : IVisionPipelineComponent {

    override fun apply(image: Mat, pipeline: VisionPipeline): Mat {
        Imgproc.rectangle(image, submat, Scalar(25.0, 175.0, 255.0), 3)
        return image
    }

}