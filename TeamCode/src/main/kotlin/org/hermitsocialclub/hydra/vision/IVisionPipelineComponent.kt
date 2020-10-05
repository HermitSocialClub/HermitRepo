package org.hermitsocialclub.hydra.vision

import org.hermitsocialclub.telecat.PersistantTelemetry
import org.opencv.core.Mat
import java.util.function.BiFunction

interface IVisionPipelineComponent : BiFunction<Mat, PersistantTelemetry, Mat>