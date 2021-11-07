package org.firstinspires.ftc.teamcode.drive;

import android.util.Pair;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;

import org.firstinspires.ftc.teamcode.vision.SkystoneVuforiaEngine;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.List;

import static java.util.Collections.emptyList;
import static org.firstinspires.ftc.teamcode.drive.Meet3Bot.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.Meet3Bot.WHEEL_BASE;

public class MecanumLocalizerEVI implements Localizer {
    MecanumDrive drive;
    SkystoneVuforiaEngine vuforia;
    Pose2d initialPose;
    public MecanumLocalizerEVI (MecanumDrive drive, SkystoneVuforiaEngine vuforia, Pose2d initialPose){
       this.drive = drive;
       this.vuforia = vuforia;
       this.initialPose = initialPose;
    }
    Pose2d poseEstimate =  new Pose2d();
    List<Double> lastWheelPositions = new ArrayList<>();
    double lastExtHeading = Double.NaN;
    Pose2d lastVPose = initialPose;
    @Override
    public void update() {
    Pose2d fullDelta = new Pose2d();
    List<Double> wheelPositions = drive.getWheelPositions();
    double extHeading = drive.getExternalHeading();
    if(!lastWheelPositions.isEmpty()){
        List<Double> wheelDeltas = new ArrayList<Double>(wheelPositions.size());
        for(int i = 0; i < wheelPositions.size(); i++) {
            wheelDeltas.set(i, wheelPositions.get(i) - lastWheelPositions.get(i));
        }
        Pose2d robotPoseDelta = MecanumKinematics.wheelToRobotVelocities(wheelDeltas,TRACK_WIDTH,WHEEL_BASE);
        double finalHeadingDelta = Angle.normDelta(extHeading-lastExtHeading);
        Pose2d eiDelta = new Pose2d(robotPoseDelta.vec(),finalHeadingDelta);
        Pair<Pose2d, Boolean> scan = vuforia.scanPose();
        if(scan.second){
            Pose2d vDelta = scan.first.minus(lastVPose);
            fullDelta = eiDelta.plus(vDelta).div(2);
            poseEstimate =
                    Kinematics.relativeOdometryUpdate(poseEstimate,
                            fullDelta);
            lastVPose = scan.first;
        }else {
            poseEstimate =
                    Kinematics.relativeOdometryUpdate(poseEstimate,
                            new Pose2d(robotPoseDelta.vec(), finalHeadingDelta));
            lastVPose = poseEstimate;
        }
        lastWheelPositions = wheelPositions;
        lastExtHeading = extHeading;
    }
    }
    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        lastWheelPositions = emptyList();
        lastExtHeading = Double.NaN;
        poseEstimate = pose2d;
    }



    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }
}
