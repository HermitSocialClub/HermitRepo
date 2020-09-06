package org.hermitsocialclub.hydra.vision

import org.opencv.core.Mat
import java.util.function.Function

interface IVisionPipelineComponent : Function<Mat, Mat>