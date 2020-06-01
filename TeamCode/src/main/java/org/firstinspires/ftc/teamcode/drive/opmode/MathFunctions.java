package org.firstinspires.ftc.teamcode.drive.opmode;

public class MathFunctions {
    /**
     * Make sure a angle is within the range of -180 to 180 degrees
     * @param angle
     * @return
     */
    public static double AngleWrap (double angle){
        while (angle< -Math.PI){
            angle += 2 * Math.PI;
        }
        while (angle > Math.PI){
            angle -= 2 * Math.PI;
        }
        return angle;
    }
}
