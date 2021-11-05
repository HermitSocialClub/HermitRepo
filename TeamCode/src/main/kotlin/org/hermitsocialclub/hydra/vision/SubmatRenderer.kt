package org.hermitsocialclub.hydra.vision

import org.opencv.core.Mat
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc

class SubmatRenderer(var submat: Rect) : IVisionPipelineComponent {

    override fun apply(image: Mat, pipeline: VisionPipeline): Mat {
        pipeline.telemetry.setData("Whole Mat Size", "%d %d", image.width(), image.height())
        pipeline.telemetry.setData("Submat Position", "%d %d", submat.x, submat.y)
        pipeline.telemetry.setData("Submat Size", "%d %d", submat.width, submat.height)

        Imgproc.rectangle(image, submat, Scalar(25.0, 175.0, 255.0), 3)
        return image
    }
}
