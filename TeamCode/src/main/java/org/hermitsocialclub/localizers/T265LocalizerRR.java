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

    private Pose2d poseOffset = new Pose2d();
    private static Pose2d mPoseEstimate = new Pose2d();
    private static com.arcrobotics.ftclib.geometry.Pose2d originalPose;
    private Pose2d rawPose = new Pose2d();
    private T265Camera.CameraUpdate up;

    public static T265Camera slamra;


    //The pose of the camera relative to the center of the robot in centimeters
    public static Transform2d slamFormPose = new Transform2d(
            new Translation2d(slamraX * .0254, slamraY * .0254),new Rotation2d(0));

    public static boolean makeCameraCenter = false;

    private static T265Camera.PoseConfidence poseConfidence;
    private static double angleModifer = 0;

    public T265LocalizerRR(HardwareMap hardwareMap) {
        new T265LocalizerRR(hardwareMap, true);
    }

    public T265LocalizerRR(HardwareMap hardwareMap, boolean resetPos) {
        poseOffset = new Pose2d();
        mPoseEstimate = new Pose2d();
        rawPose = new Pose2d();
        T265Camera tempCam  = new T265Camera(slamFormPose,.8,hardwareMap.appContext);

        if (slamra == null) {
            slamra = tempCam;
            RobotLog.d("Created Realsense Object");
            setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));
        }
        try {
            startRealsense();
        } catch (Exception ignored) {
            RobotLog.v("Realsense already started");
            if (resetPos) {
                setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));
            }
        }
        if (slamra.getLastReceivedCameraUpdate().confidence == T265Camera.PoseConfidence.Failed) {
            RobotLog.e("Realsense Failed to get Position");
        }
        originalPose = slamra.getLastReceivedCameraUpdate().pose;
    }

    /**
     * @return
     */
    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        //variable up is updated in update()
        update();
        //The FTC265 library uses Ftclib geometry, so I need to convert that to road runner GeometryS
        if (up != null) {
            Translation2d curPose = up.pose.getTranslation();
           RobotLog.v("CurPose: " + curPose.toString());
           RobotLog.v("Original Pose: " + originalPose.toString());
            Rotation2d curRot = up.pose.getRotation();
            Translation2d newPose = curPose.minus(originalPose.getTranslation());
            RobotLog.v("New Pose: " + newPose.toString());
            Rotation2d newRot = curRot.minus(originalPose.getRotation());
            //The T265's unit of measurement is meters.  dividing it by .0254 converts meters to inches.
            rawPose = new Pose2d(-newPose.getX() / .0254, -newPose.getY() / .0254, norm(newRot.getRadians() + angleModifer)); //raw pos
            RobotLog.v("Raw Pose: " + rawPose.toString());
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
         return (mPoseEstimate);

    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        update();
        originalPose = up.pose;
        RobotLog.v("Set Pose to " + pose2d.toString());
        pose2d = new Pose2d(pose2d.getX(),pose2d.getY(),0);

        RobotLog.v("SETTING POSE ESTIMATE TO " + pose2d.toString());
        poseOffset = pose2d.minus(rawPose);
        poseOffset = new Pose2d(poseOffset.getX(), poseOffset.getY(), Math.toRadians(0));
        RobotLog.v("SET POSE OFFSET TO " + poseOffset.toString());
        Pose2d newPos = new Pose2d(pose2d.getX() * .0254, pose2d.getY() * .0254, pose2d.getHeading());



         mPoseEstimate = newPos; //set mPose to new pose.
//        /* Alternate to using pose2d.minus()*/
        try {
            poseOffset = ( new Pose2d(pose2d.getX(), pose2d.getY(), pose2d.getHeading()));
        } catch (Exception e) {

        }
    }

    public static T265Camera.PoseConfidence getConfidence() {
        return poseConfidence;
    }

    /**
     * @return the heading of the robot (in radains)
     */
    public double getHeading() {
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
    private Pose2d adjustPosbyCameraPos(Pose2d mPoseEstimate)
    {
        double dist = Math.hypot(slamraX,slamraY); //distance camera is from center
        double angle = Math.atan2(slamraY,slamraX);
        double cameraAngle = mPoseEstimate.getHeading() - angle;
        double detlaX = dist * Math.cos(cameraAngle);
        double detlaY = dist * Math.sin(cameraAngle);
        return mPoseEstimate;
    }

    /**
     * Converts from FTCLib Pose to Roadrunner Pose or vice-versa
     * Returns just an object so you might want to cast things first
     * Probably would've been better as a switch block but am lazy
      */
    private Object convertPose(Object pose){

        double x = 0;
        double y = 0;
        double heading = 0;

        //Converts Roadrunner to FTCLib
        if(pose.getClass().equals(Pose2d.class)){

            x = ((Pose2d) pose).getX() * .0254;
            y = ((Pose2d) pose).getY() * .0254;
            heading = ((Pose2d) pose).getHeading();

            return new com.arcrobotics.ftclib.geometry.Pose2d(x,y,new Rotation2d(heading));
        } else if(pose.getClass().equals(com.arcrobotics.ftclib.geometry.Pose2d.class)){
            x = ((com.arcrobotics.ftclib.geometry.Pose2d) pose).getX() / .0254;
            y = ((com.arcrobotics.ftclib.geometry.Pose2d) pose).getY() / .0254;
            heading = ((com.arcrobotics.ftclib.geometry.Pose2d) pose).getHeading();

            return new Pose2d(x,y,heading);
        }else return null;

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
