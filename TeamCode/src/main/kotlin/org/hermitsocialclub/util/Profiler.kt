package org.hermitsocialclub.util

import org.hermitsocialclub.telecat.PersistantTelemetry

/**
 * A quick-and-dirty line by line profiler.
 */
class Profiler(val tag: String, val telemetry: PersistantTelemetry) {
    private var lastSwapTime = 0L
    private var section: String = ""

    fun begin(newSection: String) {
        section = newSection
        lastSwapTime = System.currentTimeMillis()
    }

    fun swap(newSection: String) {
        val now = System.currentTimeMillis()
        telemetry.setData("$tag: $section", "${now - lastSwapTime} ms")
        section = newSection
        // take currentTimeMillis again so telemetry lag isn't counted in the profiling
        lastSwapTime = System.currentTimeMillis()
    }

    fun end() {
        val now = System.currentTimeMillis()
        telemetry.setData("$tag: $section", "${now - lastSwapTime} ms")
        lastSwapTime = 0
    }
}
