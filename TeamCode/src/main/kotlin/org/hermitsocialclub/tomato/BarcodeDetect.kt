package org.hermitsocialclub.tomato

import org.hermitsocialclub.hydra.vision.IVisionPipelineComponent
import org.hermitsocialclub.hydra.vision.VisionPipeline
import org.opencv.core.Mat

class BarcodeDetect(val isRed: Boolean) : IVisionPipelineComponent {
    var result: Byte = 0

    override fun apply(mat: Mat, pipeline: VisionPipeline): Mat {
        val pt = pipeline.telemetry
        this.result = detect(mat, pipeline, isRed)

        if (result == 0.toByte()) {
            pt.setData("Barcode Scanning is a no", "")
        } else {
            if (result == 4.toByte()) {
                pt.setData("cannot find 3 red blobs", "")
            }
        }
        pt.setData("Barcode Level", result)

        return mat
    }

    private external fun detect(mat: Mat, pipeline: VisionPipeline, isRed: Boolean): Byte
}
