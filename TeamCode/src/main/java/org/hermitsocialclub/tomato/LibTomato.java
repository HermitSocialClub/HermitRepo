package org.hermitsocialclub.tomato;

public class LibTomato {
    static {
        System.loadLibrary("tomato");
    }

    public static native int splat();
}
