package org.hermitsocialclub.telecat

import com.google.gson.JsonArray
import com.google.gson.JsonObject
import com.google.gson.JsonPrimitive

/**
 * ASCIICAST BUILDER
 *
 * To create an Asciicast, build a header and print it to a file.
 * Then, for each "frame" of telemetry, build an event and append it to the file.
 *
 * Asciicast file format specification:
 * [https://github.com/asciinema/asciinema/blob/develop/doc/asciicast-v2.md](https://github.com/asciinema/asciinema/blob/develop/doc/asciicast-v2.md)
 *
 * View Asciicasts with [https://repl.it/@Arcblroth/Asciinema](https://repl.it/@Arcblroth/Asciinema)
 */
object AsciicastBuilder {

    private const val WIDTH = 20
    private const val HEIGHT = 100
    private const val CLEAR = "\u001b[2J\u001b[1;1H"

    fun buildHeader(startTime: Long, title: String?): String {
        val header = JsonObject()
        header.add("version", JsonPrimitive(2))
        header.add("width", JsonPrimitive(WIDTH))
        header.add("height", JsonPrimitive(HEIGHT))
        header.add("timestamp", JsonPrimitive(startTime))
        header.add("title", JsonPrimitive(title))
        return header.toString()
    }

    fun buildEvent(startTime: Long, currentTime: Long, data: String): String {
        val event = JsonArray()
        event.add((currentTime - startTime).toDouble() / 1000.0)
        event.add("o")
        event.add(CLEAR + data)
        return event.toString()
    }
}
