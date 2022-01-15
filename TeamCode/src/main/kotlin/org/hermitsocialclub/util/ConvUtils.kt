package org.hermitsocialclub.util

// import com.arcrobotics.ftclib.geometry.Pose2d as FLPose2d
// import com.arcrobotics.ftclib.geometry.Rotation2d
// import com.arcrobotics.ftclib.geometry.Translation2d
// import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds

private const val M2IN_FACTOR = 100.0 / 2.54
private const val IN2M_FACTOR = 0.0254

/**
 * Various math conversion utilities.
 */
object ConvUtils {
    /**
     * Converts meters to inches.
     */
    fun m2in(m: Double) = m * M2IN_FACTOR

    /**
     * Converts meters to inches.
     */
    // fun m2in(m: Translation2d) = m * M2IN_FACTOR

    /**
     * Converts inches to meters.
     */
    fun in2m(`in`: Double) = `in` * IN2M_FACTOR

    /**
     * Converts inches to meters.
     */
    fun in2m(`in`: Object) = `in` // * IN2M_FACTOR

    /**
     * Converts from a FTCLib Pose to a Roadrunner Pose.
     */
    // fun RRPose2d.toFLPose2d() = FLPose2d(in2m(this.x), in2m(this.y), Rotation2d(this.heading))

    /**
     * Converts from a Roadrunner Pose to a FTCLib Pose.
     */
    // fun FLPose2d.toRRPose2d() = RRPose2d(m2in(this.x), m2in(this.y), this.heading)

    // fun ChassisSpeeds.toRRPose2d() =
    //  RRPose2d(m2in(this.vxMetersPerSecond), m2in(this.vyMetersPerSecond), this.omegaRadiansPerSecond)
}
