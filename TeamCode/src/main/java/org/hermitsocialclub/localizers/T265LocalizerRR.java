package org.hermitsocialclub.localizers;


import static org.hermitsocialclub.drive.config.DriveConstants.slamraX;
import static org.hermitsocialclub.drive.config.DriveConstants.slamraY;
import static org.hermitsocialclub.tomato.LibTomato.SLAMRA;
import static org.hermitsocialclub.tomato.LibTomato.checkBatteryForSlamra;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.spartronics4915.lib.T265Camera;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/**
 * a Road Runner localizer that uses the Intel T265 Realsense
 */
@Config
public class T265LocalizerRR implements Localizer {

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
        checkBatteryForSlamra(hardwareMap);

        poseOffset = new Pose2d();
        mPoseEstimate = new Pose2d();
        rawPose = new Pose2d();

        update();
        if (SLAMRA.getLastReceivedCameraUpdate().confidence == T265Camera.PoseConfidence.Failed) {
            RobotLog.e("Realsense Failed to get Position");
        }
        update();
        RobotLog.e("Ran on line 78" + up.pose.toString());
        resetPose = up.pose == null ?
                new Pose2d(50,50,50)
                : up.pose;
        RobotLog.e("Reset Pose: " + resetPose);
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        //variable up is updated in update()
        update();
        //The FTC265 library uses Ftclib geometry, so I need to convert that to road runner GeometryS
        if (up != null) {
            Pose2d curPose = up.pose;
            curPose = new Pose2d(-curPose.getX(), curPose.getY(),-curPose.getHeading());
            RobotLog.d("CurPose: " + curPose.toString());
            RobotLog.d("Original Pose: " + resetPose.toString());
            Pose2d newPose = curPose.minus(new Pose2d(-resetPose.getX(), resetPose.getY(),-resetPose.getHeading()));
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
                        slamFormPose.getY(),-slamFormPose.getHeading()));
        poseOffset = new Pose2d(poseOffset.getX(), poseOffset.getY(), Math.toRadians(0));
        RobotLog.d("SET POSE OFFSET TO " + poseOffset);
        mPoseEstimate = new Pose2d(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
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
        up = SLAMRA.getLastReceivedCameraUpdate();
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
        return new Pose2d(-up.velocity.getX(),up.velocity.getY(),
        -up.velocity.getHeading());
    }

    /**
     * @param angle angle in radians
     * @return normalized angle between ranges 0 to 2π
     */
    private double norm(double angle) {
        final double TAU = 2 * Math.PI;
        return (((angle % TAU) + TAU) % TAU);
    }
}
