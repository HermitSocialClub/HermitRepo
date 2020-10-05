package org.hermitsocialclub.hydra.vision

import org.hermitsocialclub.telecat.PersistantTelemetry
import org.opencv.core.Mat
import org.openftc.easyopencv.OpenCvPipeline

class VisionPipeline(val telemetry: PersistantTelemetry, vararg components: IVisionPipelineComponent) :
    OpenCvPipeline() {

    var pipeline: MutableList<IVisionPipelineComponent> = mutableListOf(*components)

    override fun processFrame(input: Mat): Mat {
        var mat = input
        pipeline.forEach { mat = it.apply(mat, telemetry) }
        return mat
    }

}
