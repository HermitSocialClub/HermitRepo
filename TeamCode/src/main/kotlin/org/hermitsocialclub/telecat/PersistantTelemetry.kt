package org.hermitsocialclub.telecat

import org.firstinspires.ftc.robotcore.external.Telemetry
import java.util.*
import kotlin.collections.LinkedHashMap


class PersistantTelemetry @JvmOverloads constructor(val originalTelemetry: Telemetry, val log: Boolean = true) {

    private val logUUID: Long = System.currentTimeMillis()
    private val telemetryData = Collections.synchronizedMap(LinkedHashMap<String, String>())
    private val debugData = Collections.synchronizedMap(LinkedHashMap<String, String>())

    companion object {
        private const val logSeperator = "=========================="
    }

    init {
        if (log) {
            FileLogger.logToFile(
                "telemetryLog-$logUUID.cast", AsciicastBuilder.buildHeader(logUUID, "Telemetry Log")
            )
            FileLogger.clearFile("telemetryLog-latest.cast")
            FileLogger.logToFile(
                "telemetryLog-latest.cast", AsciicastBuilder.buildHeader(logUUID, "Telemetry Log")
            )
        }
    }

    /**
     * Persistently sets telemetry data.
     *
     * @param caption The "key" for this data.
     * @param value   What to set the data to.
     */
    fun setData(caption: String, value: Any) {
        telemetryData[caption] = value.toString()
        update()
    }

    fun setXData(key: String, formatKey: String, values: Array<Any>) {
        telemetryData[key] = String.format(formatKey, *values)
        update()
    }

    fun setData(key: String, formatKey: String, vararg values: Any) {
        telemetryData[key] = String.format(formatKey, *values)
        update()
    }

    fun removeData(key: String) {
        telemetryData.remove(key)
        update()
    }

    fun setDebug(key: String, value: Any) {
        debugData[key] = value.toString()
        update()
    }

    fun setXDebug(key: String, formatKey: String, values: Array<Any>) {
        debugData[key] = String.format(formatKey, *values)
        update()
    }

    fun setDebug(key: String, formatKey: String, vararg values: Any) {
        debugData[key] = String.format(formatKey, *values)
        update()
    }

    private fun update() {
        originalTelemetry.clear()
        if (log) {
            val cal = Calendar.getInstance()
            val time = String.format(Locale.CHINESE, "%tT", cal)
            val timeLength = Math.max(logSeperator.length - time.length, 0)
            val txtLogBuilder = StringBuilder()
            txtLogBuilder
                .append(logSeperator.substring(0, Math.ceil(timeLength / 2.0).toInt()))
                .append(time)
                .append(logSeperator.substring(0, Math.floor(timeLength / 2.0).toInt()))
                .append("\n")
            val logBuilder = StringBuilder()
            for ((key, value) in telemetryData) {
                txtLogBuilder.append("$key: $value")
                logBuilder.append("$key: $value\n\u001b[1G")
            }
            for ((key, value) in debugData) {
                txtLogBuilder.append("$key: $value\n")
                logBuilder.append("$key: $value\n\u001b[1G")
            }
            txtLogBuilder.append("\n")
            val event = AsciicastBuilder.buildEvent(logUUID, cal.timeInMillis, logBuilder.toString())
            FileLogger.logToFile("telemetryLog-$logUUID.cast", event)
            FileLogger.logToFile("telemetryLog-latest.cast", event)
            FileLogger.logToFile("telemetryLog-$logUUID.log", txtLogBuilder.toString())
        }
        for ((key, value) in telemetryData) {
            originalTelemetry.addData(key, value)
        }
        for ((key, value) in debugData) {
            originalTelemetry.addData(key, value)
        }
        originalTelemetry.update()
    }

    /**
     * Erases all telemetry data.
     */
    fun clear() {
        telemetryData.clear()
        originalTelemetry.clearAll()
        originalTelemetry.update()
    }

}