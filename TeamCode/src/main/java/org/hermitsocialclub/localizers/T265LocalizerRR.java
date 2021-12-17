package org.hermitsocialclub.localizers;


import static org.hermitsocialclub.drive.config.DriveConstants.slamraX;
import static org.hermitsocialclub.drive.config.DriveConstants.slamraY;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
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

    public static T265Camera slamra;

    private Pose2d poseOffset = new Pose2d();
    private Pose2d mPoseEstimate = new Pose2d();
    private com.arcrobotics.ftclib.geometry.Pose2d resetPose;
    private Pose2d rawPose = new Pose2d();
    private T265Camera.CameraUpdate up;

    /**
     * The pose of the camera relative to the center of the robot in centimeters
     */
    public Transform2d slamFormPose = new Transform2d(
            new Translation2d(slamraX * .0254, slamraY * .0254), new Rotation2d(0)
    );

    private T265Camera.PoseConfidence poseConfidence;
    private double angleModifer = 0;

    public T265LocalizerRR(HardwareMap hardwareMap) {
        new T265LocalizerRR(hardwareMap, true);
    }

    public T265LocalizerRR(HardwareMap hardwareMap, boolean resetPos) {
        poseOffset = new Pose2d();
        mPoseEstimate = new Pose2d();
        rawPose = new Pose2d();
        T265Camera tempCam = new T265Camera(slamFormPose, .8, hardwareMap.appContext);

        if (slamra == null) {
            slamra = tempCam;
            RobotLog.d("Created Realsense Object");
            setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
        }
        try {
            RobotLog.v("staring realsense");
            slamra.start();
        } catch (Exception ignored) {
            RobotLog.v("Realsense already started");
            if (resetPos) {
                setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            }
        }
        if (slamra.getLastReceivedCameraUpdate().confidence == T265Camera.PoseConfidence.Failed) {
            RobotLog.e("Realsense Failed to get Position");
        }
        resetPose = slamra.getLastReceivedCameraUpdate().pose;
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        //variable up is updated in update()
        update();
        //The FTC265 library uses Ftclib geometry, so I need to convert that to road runner GeometryS
        if (up != null) {
            Translation2d curPose = up.pose.getTranslation();
            curPose = new Translation2d(-curPose.getX(), -curPose.getY());
            RobotLog.v("CurPose: " + curPose.toString());
            RobotLog.v("Original Pose: " + resetPose.toString());
            Rotation2d curRot = up.pose.getRotation();
            Translation2d newPose = curPose.minus(new Translation2d(-resetPose.getX(), -resetPose.getY()));
            RobotLog.v("New Pose: " + newPose.toString());
            Rotation2d newRot = curRot.minus(resetPose.getRotation());
            //The T265's unit of measurement is meters.  dividing it by .0254 converts meters to inches.
            rawPose = new Pose2d(newPose.getX() / .0254, newPose.getY() / .0254, norm(newRot.getRadians() + angleModifer)); //raw pos
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
        RobotLog.v("Set Pose to " + pose2d);
        pose2d = new Pose2d(pose2d.getX(), pose2d.getY(), pose2d.getHeading());

        RobotLog.v("SETTING POSE ESTIMATE TO " + pose2d.toString());
        poseOffset = pose2d.minus(rawPose)
                .minus(new Pose2d(slamFormPose.getTranslation().getX(),
                                slamFormPose.getTranslation().getY(),0));
        poseOffset = new Pose2d(poseOffset.getX(), poseOffset.getY(), Math.toRadians(0));
        RobotLog.v("SET POSE OFFSET TO " + poseOffset);
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
     * No idea what the purpose getPoseVelocity.  Everything works fine by just using getPoseEstimate()
     * That said, the code to get the velocity is comment out below.  Haven't testing it much
     * and I don't know how well getting the velocity work or if use the velocity has any effect
     * at all.
     */
    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        //variable up is updated in update()
        ChassisSpeeds velocity = up.velocity;
        return new Pose2d(-velocity.vxMetersPerSecond / .0254, velocity.vyMetersPerSecond / .0254, velocity.omegaRadiansPerSecond);
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
