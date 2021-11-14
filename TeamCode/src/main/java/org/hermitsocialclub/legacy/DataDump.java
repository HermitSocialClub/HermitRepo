package org.hermitsocialclub.legacy;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DataDump {

    public static void dump(Exception e, Telemetry telemetry) {
        telemetry.clearAll();
        telemetry.addLine(e.getClass().getName() + ": " + e.getMessage());
        for (StackTraceElement stackTraceElement : e.getStackTrace()) {
            telemetry.addLine("     at " + stackTraceElement.toString());
        }
        telemetry.update();
    }

    public static String dump(Exception e) {
        StringBuilder dumper = new StringBuilder();
        dumper.append(e.getClass().getName() + ": " + e.getMessage());
        for (StackTraceElement stackTraceElement : e.getStackTrace()) {
            dumper.append("\n     at " + stackTraceElement.toString());
        }
        return dumper.toString();
    }

}
