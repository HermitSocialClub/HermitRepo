package org.hermitsocialclub.tomato

import org.hermitsocialclub.hydra.vision.IVisionPipelineComponent
import org.hermitsocialclub.hydra.vision.VisionPipeline
import org.opencv.core.Mat

object NativeTestPipelineComponent : IVisionPipelineComponent {
    external override fun apply(t: Mat, u: VisionPipeline): Mat
}
