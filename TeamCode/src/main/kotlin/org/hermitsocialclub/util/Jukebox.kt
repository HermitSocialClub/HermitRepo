package org.hermitsocialclub.util

import android.content.Context
import android.media.AudioAttributes
import android.media.MediaPlayer
import android.net.Uri
import android.os.Environment
import com.qualcomm.ftccommon.SoundPlayer
import com.qualcomm.robotcore.eventloop.EventLoopManager
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.robocol.TelemetryMessage
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.internal.network.NetworkConnectionHandler
import java.io.File
import java.io.IOException
import java.util.concurrent.ConcurrentHashMap

/**
 * <h1>Jukebox</h1>
 * A fast, unopinionated method for legally playing music from your FTC Robot Controller.
 *
 * @author Hermit Social Club
 * @version 1.3
 */
object Jukebox {

    private val musics = ConcurrentHashMap<String, MediaPlayer>()

    fun playSoundFromSDK(hm: HardwareMap, name: String?) {
        try {
            SoundPlayer.getInstance().startPlaying(
                hm.appContext,
                hm.appContext.resources.getIdentifier(name, "raw", hm.appContext.packageName)
            )
        } catch (e: Exception) {
        }
    }

    @JvmOverloads
    fun playSoundFromAndroid(c: Context, filename: String, repeat: Boolean = false) {
        try {
            if (musics.contains(filename)) {
                musics[filename]!!.stop()
                musics.remove(filename)
            }

            val mediaPlayer = MediaPlayer()
            mediaPlayer.setAudioAttributes(
                AudioAttributes.Builder()
                    .setContentType(AudioAttributes.CONTENT_TYPE_MUSIC)
                    .setUsage(AudioAttributes.USAGE_GAME)
                    .build()
            )

            //Attempt to play the file from /storage/emulated/0/JukeboxExternalStorage if it exists
            //Otherwise play from TeamCode/res/raw
            val path =
                File(Environment.getExternalStorageDirectory().path + File.separator + "JukeboxExternalStorage" + File.separator + filename + ".wav")
            if (path.exists()) {
                mediaPlayer.setDataSource(path.path)
            } else {
                mediaPlayer.setDataSource(c, Uri.parse("android.resource://" + c.packageName + "/raw/" + filename))
            }

            mediaPlayer.setVolume(1.0f, 1.0f)
            mediaPlayer.prepare()
            mediaPlayer.isLooping = repeat
            mediaPlayer.start()
            musics[filename] = mediaPlayer
        } catch (e: IOException) {
            setTelemetryWarning(e.toString())
        }
    }

    fun setTelemetryWarning(s: String) {
        RobotLog.addGlobalWarningMessage(s)
        try {
            val telemetry = TelemetryMessage()
            telemetry.tag = EventLoopManager.SYSTEM_WARNING_KEY
            telemetry.addData(EventLoopManager.SYSTEM_WARNING_KEY, s)
            NetworkConnectionHandler.getInstance().sendDataToPeer(telemetry)
        } catch (e2: Exception) {
            e2.printStackTrace()
        }
    }

    fun destroy() {
        musics.forEach {
            it.value.stop()
        }
        musics.clear()
    }

}