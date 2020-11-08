package org.hermitsocialclub.hydra.vision

import com.qualcomm.robotcore.hardware.HardwareMap
import org.hermitsocialclub.telecat.PersistantTelemetry
import org.opencv.core.Mat
import org.openftc.easyopencv.OpenCvPipeline

class VisionPipeline(
    val hardwareMap: HardwareMap,
    val telemetry: PersistantTelemetry,
    vararg components: IVisionPipelineComponent
) : OpenCvPipeline() {

    private val pipeline: List<IVisionPipelineComponent> = listOf(*components)
    var cannyLowerThreshold: Double = 35.0
    var cannyUpperThreshold: Double = 125.0

    init {
        pipeline.forEach {
            it.init(this)
        }
    }

    override fun processFrame(input: Mat): Mat {
        var mat = input
        pipeline.forEach { mat = it.apply(mat, this) }
        return mat
    }

}
