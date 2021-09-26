package org.hermitsocialclub.tomato;

import android.content.Context;
import com.qualcomm.robotcore.eventloop.opmode.AnnotatedOpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.util.RobotLog;
import org.openftc.opencvrepackaged.DynamicOpenCvNativeLibLoader;

import java.io.PrintWriter;
import java.io.StringWriter;

public class LibTomato {
    /**
     * The LibTomato init function.
     * @throws UnsatisfiedLinkError if the library could not be loaded.
     * @throws AssertionError if {@link #splat} did not return a funny number.
     */
    @OpModeRegistrar
    public static void init(Context context, AnnotatedOpModeManager manager) {
        try {
            // make sure OpenCV is loaded first
            DynamicOpenCvNativeLibLoader.loadNativeLibOnStartRobot(context, manager);
            // load Tomato
            System.loadLibrary("tomato");
        } catch(UnsatisfiedLinkError e) {
            StringWriter writer = new StringWriter();
            e.printStackTrace(new PrintWriter(writer));
            RobotLog.ee("LibTomato", e, "Tomato init failed!");
            RobotLog.addGlobalWarningMessage("Tomato init failed: " + writer);
            throw e;
        }

        if(splat() != 69) {
            throw new AssertionError("Library init failed: splat() was not nice!");
        }
    }

    public static native int splat();
}
