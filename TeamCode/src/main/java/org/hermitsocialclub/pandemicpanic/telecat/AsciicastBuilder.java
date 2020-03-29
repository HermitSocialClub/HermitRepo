package org.hermitsocialclub.pandemicpanic.telecat;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonPrimitive;

/*
 * Arcblroth presents...
 *
 * ASCIICAST BUILDER
 *
 * To create an Asciicast, build a header and print it to a file.
 * Then, for each "frame" of telemetry, build an event and append it to the file.
 *
 * Asciicast file format specification:
 *   https://github.com/asciinema/asciinema/blob/develop/doc/asciicast-v2.md
 *
 * View Asciicasts with https://repl.it/@Arcblroth/Asciinema
 *
 */
public class AsciicastBuilder {

    private static final int WIDTH = 20;
    private static final int HEIGHT = 100;
    private static final String CLEAR = "\u001b[2J\u001b[1;1H";

    public static String buildHeader(long startTime, String title) {
        JsonObject header = new JsonObject();
        header.add("version", new JsonPrimitive(2));
        header.add("width", new JsonPrimitive(WIDTH));
        header.add("height", new JsonPrimitive(HEIGHT));
        header.add("timestamp", new JsonPrimitive(startTime));
        header.add("title", new JsonPrimitive(title));
        return header.toString();
    }

    public static String buildEvent(long startTime, long currentTime, String data) {
        JsonArray event = new JsonArray();
        event.add(((double) (currentTime - startTime)) / 1000D);
        event.add("o");
        event.add(CLEAR + data);
        return event.toString();
    }

}
