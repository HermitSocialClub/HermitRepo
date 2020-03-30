package org.hermitsocialclub.pandemicpanic.telecat;

import android.os.Environment;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class FileLogger {

    public static void clearFile(String filepath) {
        try {
            File file = new File(Environment.getExternalStorageDirectory().getPath() + File.separator + "DataDumpLogs" + File.separator + filepath);
            file.getParentFile().mkdirs();
            if (!file.exists()) file.createNewFile();
            FileWriter writer = new FileWriter(file);
            writer.write("");
            writer.flush();
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void logToFile(String filepath, String nextLineToLog) {
        try {
            File file = new File(Environment.getExternalStorageDirectory().getPath() + File.separator + "DataDumpLogs" + File.separator + filepath);
            file.getParentFile().mkdirs();
            if (!file.exists()) file.createNewFile();
            FileWriter writer = new FileWriter(file, true);
            writer.append(nextLineToLog);
            writer.flush();
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
