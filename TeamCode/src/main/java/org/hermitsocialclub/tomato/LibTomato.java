package org.hermitsocialclub.tomato;

import android.content.Context;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.AnnotatedOpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.spartronics4915.lib.T265Camera;
import com.spartronics4915.lib.T265Helper;
import org.hermitsocialclub.drive.config.DriveConstants;
import org.hermitsocialclub.util.Jukebox;
import org.openftc.opencvrepackaged.DynamicOpenCvNativeLibLoader;
import org.tensorflow.lite.TensorFlowLite;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.Locale;

public class LibTomato {
    private static volatile boolean INIT_YET = false;

    //Removed the camera initialization from libtomato to check if it is a source of current localization issues
    //public static T265Camera SLAMRA;

    /**
     * The LibTomato init function.
     *
     * @throws UnsatisfiedLinkError if the library could not be loaded.
     * @throws AssertionError       if {@link #splat} did not return a funny number.
     */
    @OpModeRegistrar
    public static void init(Context context, AnnotatedOpModeManager manager) {
        // prevent double initializations
        if (INIT_YET) return;

        try {
            // make sure TensorFlowLite and OpenCV are loaded first
            TensorFlowLite.init();
            DynamicOpenCvNativeLibLoader.loadNativeLibOnStartRobot(context, manager);
            // load Tomato
            System.loadLibrary("tomato");
        } catch (UnsatisfiedLinkError e) {
            StringWriter writer = new StringWriter();
            e.printStackTrace(new PrintWriter(writer));
            RobotLog.ee("LibTomato", e, "Tomato init failed!");
            RobotLog.addGlobalWarningMessage("Tomato init failed: " + writer);
            throw e;
        }

        // Do a quick test to make sure the library linked properly
        if (splat() != 12675) {
            throw new AssertionError("Library init failed: splat() did not return expected value!");
        }

        // Load the T265 camera
        /*try {
            SLAMRA = T265Helper.getCamera(
                    new T265Camera.OdometryInfo(new Pose2d(DriveConstants.slamraX, DriveConstants.slamraY), 0.0),
                    context
            );
            if (!SLAMRA.isStarted()) {
                SLAMRA.start();
            }
        } catch (Throwable t) {
            StringWriter writer = new StringWriter();
            t.printStackTrace(new PrintWriter(writer));
            RobotLog.ee("LibTomato", t, "FTC265 init failed!");
            RobotLog.addGlobalWarningMessage("FTC265 init failed. All OpModes using the SLAMRA camera will crash.");
        }*/

        // we're done loading!
        INIT_YET = true;
    }

    public static void checkBatteryForSlamra(HardwareMap hardwareMap) {
        hardwareMap.voltageSensor.entrySet().stream()
                .mapToDouble(s -> s.getValue().getVoltage())
                .filter(v -> v > 0)
                .min()
                .ifPresent(batteryLevel -> {
                    if (batteryLevel < 12) {
                        Jukebox.INSTANCE.setTelemetryWarning(String.format(
                                Locale.ROOT,
                                "SLAMRA will probably not run correctly (battery voltage %.2fV < 12V)!",
                                batteryLevel
                        ));
                    }
                });
    }

    public static native int splat();

    public static native void panicTest();
}
