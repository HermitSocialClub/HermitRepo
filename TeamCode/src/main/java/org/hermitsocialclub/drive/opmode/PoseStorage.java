package org.hermitsocialclub.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

// A hacky way to transfer data between auto and teleop
public class PoseStorage {

    public static Pose2d currentPose = new Pose2d();

}
