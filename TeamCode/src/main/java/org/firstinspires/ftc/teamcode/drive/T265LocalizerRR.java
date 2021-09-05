package org.firstinspires.ftc.teamcode.drive;
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

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/**
 * a Road Runner localizer that uses the Intel T265 Realsense
 */
@Config
public class T265LocalizerRR implements Localizer {

    private Pose2d poseOffset = new Pose2d();
    private static Pose2d mPoseEstimate = new Pose2d();
    private Pose2d rawPose = new Pose2d();
    private T265Camera.CameraUpdate up;

    public static T265Camera slamra;

    public static double slamraX = -9;
    public static double slamraY = 0;

    public static boolean makeCameraCenter = true;

    private static T265Camera.PoseConfidence poseConfidence;
    private static double angleModifer = 0;

    public T265LocalizerRR(HardwareMap hardwareMap) {
        new T265LocalizerRR(hardwareMap, true);
    }

    public T265LocalizerRR(HardwareMap hardwareMap, boolean resetPos) {
        poseOffset = new Pose2d();
        mPoseEstimate = new Pose2d();
        rawPose = new Pose2d();

        if (slamra == null) {
            slamra = FtcRobotControllerActivity.slamra;
            RobotLog.d("Created Realsense Object");
            setPoseEstimate(new Pose2d(0,0,0));
        }
        try {
            startRealsense();
        } catch (Exception ignored) {
            RobotLog.v("Realsense already started");
            if (resetPos) {
                slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(0,0, new Rotation2d(0)));
            }
        }
        if (slamra.getLastReceivedCameraUpdate().confidence == T265Camera.PoseConfidence.Failed) {
            RobotLog.e("Realsense Failed to get Position");
        }
    }

    /**
     * @return
     */
    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        //variable up is updated in update()

        //The FTC265 library uses Ftclib geometry, so I need to convert that to road runner Geometry
        if (up != null) {
            Translation2d oldPose = up.pose.getTranslation();
            Rotation2d oldRot = up.pose.getRotation();
            //The T265's unit of measurement is meters.  dividing it by .0254 converts meters to inches.
            rawPose = new Pose2d(oldPose.getX() / .0254, oldPose.getY() / .0254, norm(oldRot.getRadians() + angleModifer)); //raw pos
            mPoseEstimate = rawPose.plus(poseOffset); //offsets the pose to be what the pose estimate is;
        } else {
            RobotLog.v("NULL Camera Update");
        }

//       try {
//           if (up.confidence == T265Camera.PoseConfidence.Failed) {
//               RobotLog.setGlobalWarningMessage("Realsense Failed");
//           }
//       } catch (Exception e) {
//           RobotLog.setGlobalWarningMessage("Realsense Might have failed");
//       }
//        RobotLog.v("Raw POS: " + rawPose.toString());
//        RobotLog.v("POSE OFFSET " + poseOffset.toString());
//        RobotLog.v("POSE ESTIMATE " + mPoseEstimate.toString());
        if (makeCameraCenter) return mPoseEstimate;
        else {
            Pose2d robotCenter = adjustPosbyCameraPos();
            RobotLog.v("robot Center pos: " + robotCenter.toString());
            return robotCenter;
        }
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        RobotLog.v("Set Pose to " + pose2d.toString());
        //  Pose2d newPos = new Pose2d(pose2d.getX() * .0254, pose2d.getY() * .0254, pose2d.getHeading());
        // slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(newPos.getX(), newPos.getY(), new Rotation2d(newPos.getHeading())));
        pose2d = new Pose2d(pose2d.getX(),pose2d.getY(),0);
        RobotLog.v("SETTING POSE ESTIMATE TO " + pose2d.toString());
        poseOffset = pose2d.minus(rawPose);
        poseOffset = new Pose2d(poseOffset.getX(), poseOffset.getY(), Math.toRadians(0));
        RobotLog.v("SET POSE OFFSET TO " + poseOffset.toString());
        // mPoseEstimate = rawPose.plus(poseOffset); //set mPose to new pose.
//        /* Alternate to using pose2d.minus()*/
//        try {
//            poseOffset = new Pose2d(pose2d.getX() - rawPose.getX(), pose2d.getY() - rawPose.getY(), pose2d.getHeading() - rawPose.getHeading());
//        } catch (Exception e) {
//
//        }
    }

    public static T265Camera.PoseConfidence getConfidence() {
        return poseConfidence;
    }

    /**
     * @return the heading of the robot (in radains)
     */
    public static double getHeading() {
        return norma(mPoseEstimate.getHeading() - angleModifer);
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
     No idea what the purpose getPoseVelocity.  Everything works fine by just using getPoseEstimate()
     That said, the code to get the velocity is comment out below.  Haven't testing it much
     and I don't know how well getting the velocity work or if use the velocity has any effect
     at all.
     */
    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        //variable up is updated in update()

        ChassisSpeeds velocity = up.velocity;
        return new Pose2d(velocity.vxMetersPerSecond /.0254,velocity.vyMetersPerSecond /.0254,velocity.omegaRadiansPerSecond);
    }

    /**
     * @param angle angle in radians
     * @return normiazled angle between ranges 0 to 2Pi
     */
    private double norm(double angle)
    {
        while (angle>Math.toRadians(360)) angle-=Math.toRadians(360);
        while (angle<=0) angle+=Math.toRadians(360);
        return angle;
    }
    private static double norma(double angle)
    {
        while (angle>Math.toRadians(360)) angle-=Math.toRadians(360);
        while (angle<=0) angle+=Math.toRadians(360);
        return angle;
    }

    /**
     * DO NOT USE THiS
     */
    @Deprecated
    @SuppressWarnings("SpellCheckingInspection")
    private Pose2d adjustPosbyCameraPos()
    {
        double dist = Math.hypot(slamraX,slamraY); //distance camera is from center
        double angle = Math.atan2(slamraY,slamraX);
        double cameraAngle = mPoseEstimate.getHeading() - angle;
        double detlaX = dist * Math.cos(cameraAngle);
        double detlaY = dist * Math.sin(cameraAngle);
        return mPoseEstimate.minus(new Pose2d(detlaX,detlaY));
    }

    /**
     * starts realsense
     * (Called automatically when a program using this starts)
     */
    /*
    Unused methods.  Here just in case they may be needed.
     */
    @Deprecated
    public static void startRealsense()
    {
        RobotLog.v("staring realsense");
        slamra.start();
    }

    /**
     * stops the realsense
     * (called automatically when a program stops)
     */
    @Deprecated
    public static void stopRealsense()
    {
        RobotLog.v("Stopping Realsense");
        slamra.stop();
    }
}
