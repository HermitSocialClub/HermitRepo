package org.hermitsocialclub.telecat

import android.os.Environment
import java.io.File
import java.io.FileWriter
import java.io.IOException

object FileLogger {

    fun clearFile(filepath: String) {
        try {
            val file =
                File(Environment.getExternalStorageDirectory().path + File.separator + "DataDumpLogs" + File.separator + filepath)
            file.parentFile.mkdirs()
            if (!file.exists()) file.createNewFile()
            val writer = FileWriter(file)
            writer.write("")
            writer.flush()
            writer.close()
        } catch (e: IOException) {
            e.printStackTrace()
        }
    }

    fun logToFile(filepath: String, nextLineToLog: String?) {
        try {
            val file =
                File(Environment.getExternalStorageDirectory().path + File.separator + "DataDumpLogs" + File.separator + filepath)
            file.parentFile.mkdirs()
            if (!file.exists()) file.createNewFile()
            val writer = FileWriter(file, true)
            writer.append(nextLineToLog)
            writer.flush()
            writer.close()
        } catch (e: IOException) {
            e.printStackTrace()
        }
    }
}