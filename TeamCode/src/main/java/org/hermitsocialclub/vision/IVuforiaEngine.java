package org.hermitsocialclub.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Vec3F;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

public interface IVuforiaEngine {

    void init(HardwareMap ahwMap);

    OpenGLMatrix getPose(String id);

    Vec3F getPosition(String id);

    /**
     *
     * Gets the distance from the camera to a tracked object, in millimeters.
     * If the object is not visible, this returns {@link Double#POSITIVE_INFINITY}<br/>
     * <br/>
     * Thanks to Mrs Shaw for helping us understand matrices.<br/>
     * Thanks to <a href="https://youtu.be/vlb3P7arbkU?t=208">this video</a> for helping us understand homegenous transformation matrices.</br>
     *
     * @param id String id of a tracked object.
     * @return distance in millimeters.
     *
     */
    Double getDistanceToObject(String id);

}
