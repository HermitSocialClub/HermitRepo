package org.hermitsocialclub.hydra.vision

import org.opencv.core.Mat
import org.openftc.easyopencv.OpenCvPipeline

class VisionPipeline(vararg components: IVisionPipelineComponent) : OpenCvPipeline() {

    var pipeline: MutableList<IVisionPipelineComponent> = mutableListOf()

    init {
        pipeline.addAll(components)
    }

    override fun processFrame(input: Mat): Mat {
        var mat = input
        pipeline.forEach { mat = it.apply(mat) }
        return mat
    }

}
