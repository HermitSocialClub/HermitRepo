package org.hermitsocialclub.tomato;

import android.content.Context;
import com.qualcomm.robotcore.eventloop.opmode.AnnotatedOpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.util.RobotLog;
import org.openftc.opencvrepackaged.DynamicOpenCvNativeLibLoader;
import org.tensorflow.lite.TensorFlowLite;

import java.io.PrintWriter;
import java.io.StringWriter;

public class LibTomato {
    private static volatile boolean INIT_YET = false;

    /**
     * The LibTomato init function.
     * @throws UnsatisfiedLinkError if the library could not be loaded.
     * @throws AssertionError if {@link #splat} did not return a funny number.
     */
    @OpModeRegistrar
    public static void init(Context context, AnnotatedOpModeManager manager) {
        // prevent double initializations
        if(INIT_YET) return;

        try {
            // make sure TensorFlowLite and OpenCV are loaded first
            TensorFlowLite.init();
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

        // we're done loading!
        INIT_YET = true;
    }

    public static native int splat();
}
