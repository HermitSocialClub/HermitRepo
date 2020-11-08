package org.hermitsocialclub.hydra.vision

import org.opencv.core.Mat
import java.util.function.BiFunction

interface IVisionPipelineComponent : BiFunction<Mat, VisionPipeline, Mat> {
    fun init(visionPipeline: VisionPipeline) {}
}