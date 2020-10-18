package org.hermitsocialclub.hydra.vision

import org.hermitsocialclub.telecat.PersistantTelemetry
import org.opencv.core.Mat
import org.openftc.easyopencv.OpenCvPipeline

class VisionPipeline(val telemetry: PersistantTelemetry, vararg components: IVisionPipelineComponent) :
    OpenCvPipeline() {

    private val pipeline: MutableList<IVisionPipelineComponent> = mutableListOf(*components)
    var cannyLowerThreshold: Double = 35.0
    var cannyUpperThreshold: Double = 125.0

    override fun processFrame(input: Mat): Mat {
        var mat = input
        pipeline.forEach { mat = it.apply(mat, this) }
        return mat
    }

}
