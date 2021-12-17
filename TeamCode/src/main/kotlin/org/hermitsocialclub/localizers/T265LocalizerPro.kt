package org.hermitsocialclub.localizers

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.geometry.Transform2d
import com.arcrobotics.ftclib.geometry.Translation2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.RobotLog
import com.spartronics4915.lib.T265Camera
import org.hermitsocialclub.drive.config.DriveConstants
import org.hermitsocialclub.util.ConvUtils.in2m
import org.hermitsocialclub.util.ConvUtils.toRRPose2d

/**
 * Pose of the camera relative to the center of the robot (in **meters**)
 */
val ROBOT_OFFSET = Transform2d(
    in2m(Translation2d(DriveConstants.slamraX, DriveConstants.slamraY)),
    Rotation2d(0.0),
)

@Suppress("SpellCheckingInspection")
var SLAMRA: T265Camera? = T265LocalizerRR.slamra

class T265LocalizerPro(hardwareMap: HardwareMap) : Localizer {
    init {
        if (SLAMRA == null) {
            RobotLog.d("Creating Realsense Object")
            T265Camera(ROBOT_OFFSET, .8, hardwareMap.appContext).also {
                SLAMRA = it
            }
        } else {
            SLAMRA!!
        }.let {
            // Start the camera if it's not already started
            if(!it.isStarted) {
                RobotLog.v("Staring Realsense")
                it.start()
            }

            // Check the camera's self-esteem
            if (it.lastReceivedCameraUpdate.confidence == T265Camera.PoseConfidence.Failed) {
                RobotLog.e("Realsense Failed to get Position")
            }
        }
    }

    /**
     * The offset between what the camera reports and
     * where RoadRunner says the robot should be.
     */
    private var offset = Pose2d()

    override var poseEstimate: Pose2d
        get() = SLAMRA!!.lastReceivedCameraUpdate.pose.toRRPose2d() + offset
        set(value) {
            offset = value - poseEstimate
        }

    override val poseVelocity: Pose2d
        get() = SLAMRA!!.lastReceivedCameraUpdate.velocity.toRRPose2d()

    // T265Camera updates by itself, so do nothing here
    override fun update() = Unit
}
