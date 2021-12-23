package org.hermitsocialclub.localizers;


import static org.checkerframework.checker.units.UnitsTools.min;
import static org.hermitsocialclub.drive.config.DriveConstants.slamraX;
import static org.hermitsocialclub.drive.config.DriveConstants.slamraY;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.spartronics4915.lib.T265Camera;
import com.spartronics4915.lib.T265Helper;

import org.hermitsocialclub.drive.config.DriveConstants;
import org.hermitsocialclub.util.Jukebox;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.function.Consumer;

/**
 * a Road Runner localizer that uses the Intel T265 Realsense
 */
@Config
public class T265LocalizerRR implements Localizer {

    public static T265Camera slamra;

    private Pose2d poseOffset = new Pose2d();
    private Pose2d mPoseEstimate = new Pose2d();
    public Pose2d resetPose = new Pose2d();
    private Pose2d rawPose = new Pose2d();
    private T265Camera.CameraUpdate up;

    /**
     * The pose of the camera relative to the center of the robot in centimeters
     */
    public Pose2d slamFormPose = new Pose2d(slamraX, slamraY, 0);

    private T265Camera.PoseConfidence poseConfidence;
    private double angleModifer = 0;

    public T265LocalizerRR(HardwareMap hardwareMap) {
        new T265LocalizerRR(hardwareMap, true);
    }

    public T265LocalizerRR(HardwareMap hardwareMap, boolean resetPos) {
        poseOffset = new Pose2d();
        mPoseEstimate = new Pose2d();
        rawPose = new Pose2d();
        slamra = T265Helper.getCamera(
                new T265Camera.OdometryInfo(new Pose2d(slamraX, slamraY,0), 0.0),
                hardwareMap.appContext
        );
        RobotLog.d("Created Realsense Object");
        setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
        if (slamra == null) {
            // Force an update here even if the camera is not
            // ready yet to prevent NullPointerExceptions

        }
        try {
            RobotLog.d("starting realsense");
            slamra.setPose(new Pose2d(0,0,0));
            slamra.start();
        } catch (Exception ignored) {
            RobotLog.v("Realsense already started");
            if (resetPos) {
                setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            }
        }
        update();
        if (slamra.getLastReceivedCameraUpdate().confidence == T265Camera.PoseConfidence.Failed) {
            RobotLog.e("Realsense Failed to get Position");
        }
        update();
        RobotLog.e("Ran on line 78" + up.pose.toString());
        resetPose = up.pose == null ?
                new Pose2d(50,50,50)
                : up.pose;
        RobotLog.e("Reset Pose: " + resetPose.toString());
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        //variable up is updated in update()
        update();
        //The FTC265 library uses Ftclib geometry, so I need to convert that to road runner GeometryS
        if (up != null) {
            Pose2d curPose = up.pose;
            curPose = new Pose2d(-curPose.getX(), curPose.getY());
            RobotLog.d("CurPose: " + curPose.toString());
            RobotLog.d("Original Pose: " + resetPose.toString());
            Pose2d newPose = curPose.minus(new Pose2d(-resetPose.getX(), resetPose.getY(),resetPose.getHeading()));
            RobotLog.d("New Pose: " + newPose.toString());
            //The T265's unit of measurement is meters.  dividing it by .0254 converts meters to inches.
            rawPose = new Pose2d(newPose.getX(), newPose.getY(), norm(newPose.getHeading() + angleModifer)); //raw pos
            RobotLog.v("Raw Pose: " + rawPose);
            mPoseEstimate = rawPose.plus(poseOffset); // offsets the pose to be what the pose estimate is
        } else {
            RobotLog.v("NULL Camera Update");
        }
        return mPoseEstimate;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        update();
        resetPose = up.pose;
        RobotLog.d("Pose set at" + resetPose.toString());
        RobotLog.d("Set Pose to " + pose2d);
        pose2d = new Pose2d(pose2d.getX(), pose2d.getY(), pose2d.getHeading());

        RobotLog.d("SETTING POSE ESTIMATE TO " + pose2d.toString());
        poseOffset = pose2d.minus(rawPose)
                .minus(new Pose2d(slamFormPose.getX(),
                        slamFormPose.getY(),0));
        poseOffset = new Pose2d(poseOffset.getX(), poseOffset.getY(), Math.toRadians(0));
        RobotLog.d("SET POSE OFFSET TO " + poseOffset);
        mPoseEstimate = new Pose2d(pose2d.getX() * .0254, pose2d.getY() * .0254, pose2d.getHeading());
        poseOffset = new Pose2d(pose2d.getX(), pose2d.getY(), pose2d.getHeading()); //?
    }

    public T265Camera.PoseConfidence getConfidence() {
        return poseConfidence;
    }

    /**
     * @return the heading of the robot (in radians)
     */
    public double getHeading() {
        return norm(mPoseEstimate.getHeading() - angleModifer);
    }

    /**
     * updates the camera.  Used in
     * nowhere
     */
    @Override
    public void update() {
        up = slamra.getLastReceivedCameraUpdate();
        poseConfidence = up.confidence;
    }

    /**
     * Get the current velocity of the T265.
     *
     * @return the velocity
     */
    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return up.velocity;
    }

    /**
     * @param angle angle in radians
     * @return normalized angle between ranges 0 to 2π
     */
    private double norm(double angle) {
        final double TAU = 2 * Math.PI;
        return (((angle % TAU) + TAU) % TAU);
    }

    /**
     * Stops the realsense
     */
    public static void stopRealsense() {
        RobotLog.v("Stopping Realsense");
        slamra.stop();
    }
}
