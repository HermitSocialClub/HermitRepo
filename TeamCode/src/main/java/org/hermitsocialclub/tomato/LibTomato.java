package org.hermitsocialclub.tomato;

import org.hermitsocialclub.util.Jukebox;

import java.io.PrintWriter;
import java.io.StringWriter;

public class LibTomato {
    static {
        try {
            System.loadLibrary("tomato");
        } catch(UnsatisfiedLinkError e) {
            StringWriter writer = new StringWriter();
            e.printStackTrace(new PrintWriter(writer));
            Jukebox.INSTANCE.setTelemetryWarning(writer.toString());
            throw new RuntimeException(e);
        }
    }

    /**
     * The LibTomato init function.
     * @throws UnsatisfiedLinkError if the library could not be loaded.
     * @throws AssertionError if {@link #splat} did not return a funny number.
     */
    public static void init() throws UnsatisfiedLinkError {
        if(splat() != 69) {
            throw new AssertionError("Library init failed: splat() was not nice!");
        }
    }

    public static native int splat();
}
