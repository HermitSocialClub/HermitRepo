package org.hermitsocialclub.hydra.vision

import org.opencv.core.Mat

/**
 * Because the camera loop runs async
 * to the main loop, we need this to
 * sync data between the two.
 */
class VisionSemaphore : IVisionPipelineComponent {

    private val lock = Object()

    override fun apply(t: Mat, u: VisionPipeline): Mat {
        synchronized(lock) {
            lock.notifyAll()
        }
        return t
    }

    /**
     * Blocks indefinitely until the next frame.
     */
    fun waitForFrame() {
        synchronized(lock) {
            lock.wait()
        }
    }

}
