package org.hermitsocialclub.pandemicpanic.telecat;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Calendar;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class PersistantTelemetry {

    private final Telemetry t;
    private static final String logSeperator = "==========================";
    private final long logUUID;
    private final boolean log;
    private ConcurrentHashMap<String, String> telemetryData = new ConcurrentHashMap<String, String>();
    private ConcurrentHashMap<String, String> debugData = new ConcurrentHashMap<String, String>();

    public PersistantTelemetry(Telemetry t) {
        this(t, true);
    }

    public PersistantTelemetry(Telemetry t, boolean log) {
        this.t = t;
        this.logUUID = System.currentTimeMillis();
        this.log = log;
        if (log) {
            FileLogger.logToFile("telemetryLog-" + logUUID + ".cast",
                    AsciicastBuilder.buildHeader(logUUID, "Telemetry Log") + "\n");
            FileLogger.clearFile("telemetryLog-latest.cast");
            FileLogger.logToFile("telemetryLog-latest.cast",
                    AsciicastBuilder.buildHeader(logUUID, "Telemetry Log") + "\n");
        }
    }

    /**
     * Persistently sets telemetry data.
     *
     * @param caption The "key" for this data.
     * @param value   What to set the data to.
     */
    public void setData(String caption, Object value) {
        //if(!telemetryData.contains(caption)) {
        telemetryData.put(caption, value.toString());
        //}
        update();
    }

    public void setXData(String key, String formatKey, Object[] values) {
        telemetryData.put(key, String.format(formatKey, values));
        update();
    }

    public void setData(String key, String formatKey, Object... values) {
        telemetryData.put(key, String.format(formatKey, values));
        update();
    }

    public void removeData(String key) {
        telemetryData.remove(key);
        update();
    }

    public void setDebug(String key, Object value) {
        debugData.put(key, value.toString());
        update();
    }

    public void setXDebug(String key, String formatKey, Object[] values) {
        debugData.put(key, String.format(formatKey, values));
        update();
    }

    public void setDebug(String key, String formatKey, Object... values) {
        debugData.put(key, String.format(formatKey, values));
        update();
    }

    private void update() {
        t.clear();
        if (log) {
            Calendar cal = Calendar.getInstance();
            String time = String.format(Locale.CHINESE, "%tT", cal);
            int timeLength = Math.max(logSeperator.length() - time.length(), 0);
            StringBuilder txtLogBuilder = new StringBuilder(
                    logSeperator.substring(0, (int) Math.ceil(timeLength / 2D))
                            + time
                            + logSeperator.substring(0, (int) Math.floor(timeLength / 2D))
                            + "\n");
            StringBuilder logBuilder = new StringBuilder();
            for (Map.Entry<String, String> e : telemetryData.entrySet()) {
                txtLogBuilder.append(e.getKey() + ": " + e.getValue() + "\n");
                logBuilder.append(e.getKey() + ": " + e.getValue() + "\n\u001b[1G");
            }
            for (Map.Entry<String, String> d : debugData.entrySet()) {
                txtLogBuilder.append(d.getKey() + ": " + d.getValue() + "\n");
                logBuilder.append(d.getKey() + ": " + d.getValue() + "\n\u001b[1G");
            }
            String event = AsciicastBuilder.buildEvent(logUUID, cal.getTimeInMillis(), logBuilder.toString()) + "\n";
            FileLogger.logToFile("telemetryLog-" + logUUID + ".cast", event);
            FileLogger.logToFile("telemetryLog-latest.cast", event);
            FileLogger.logToFile("telemetryLog-" + logUUID + ".log", txtLogBuilder.toString());
        }
        for (Map.Entry<String, String> e : telemetryData.entrySet()) {
            t.addData(e.getKey(), e.getValue());
        }
        for (Map.Entry<String, String> d : debugData.entrySet()) {
            t.addData(d.getKey(), d.getValue());
        }
        t.update();
    }

    /**
     * Erases all telemetry data.
     */
    public void clear() {
        telemetryData.clear();
        t.clearAll();
        t.update();
    }

    public Telemetry getOriginalTelemetry() {
        return t;
    }

}
