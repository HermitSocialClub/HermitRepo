package org.hermitsocialclub.tomato

import org.hermitsocialclub.hydra.vision.IVisionPipelineComponent
import org.hermitsocialclub.hydra.vision.VisionPipeline
import org.opencv.core.Mat

class DuckDetect() : IVisionPipelineComponent {
    var result: Byte = 0

    internal enum class DUCKPOS {
        LEFT, CENTER, RIGHT
    }

    override fun apply(mat: Mat, pipeline: VisionPipeline): Mat {
        val pt = pipeline.telemetry
        this.result = duckDetector(mat, pipeline)

//        if (result == 0.toByte()) {
//            pt.setData("Barcode Scanning is a no", "")
//        } else {
//            if (result == 4.toByte()) {
//                pt.setData("cannot find 3 red blobs", "")
//            }
//        }
        pt.setData("duckposition", DUCKPOS.values()[result - 1])

        return mat
    }

    private external fun duckDetector(mat: Mat, pipeline: VisionPipeline): Byte
}
