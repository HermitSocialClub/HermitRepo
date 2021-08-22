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

    public static native int splat();
}
