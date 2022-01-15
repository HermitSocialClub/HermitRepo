package org.hermitsocialclub.hydra.vision

import org.opencv.core.Mat

/**
 * Because the camera loop runs async
 * to the main loop, we need this to
 * make sure at least 1 frame has been
 * processed before we query the result.
 */
class FirstFrameSemaphore : IVisionPipelineComponent {

    private var firstFramePassed = false
    private val lock = Object()

    override fun apply(t: Mat, u: VisionPipeline): Mat {
        synchronized(lock) {
            if (!firstFramePassed) {
                lock.notifyAll()
                firstFramePassed = true
            }
        }
        return t
    }

    /**
     * Blocks indefinitely until the first frame.
     */
    fun waitForFirstFrame() {
        synchronized(lock) {
            if (!firstFramePassed) {
                lock.wait()
            }
        }
    }
}
