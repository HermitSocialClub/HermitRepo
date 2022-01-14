package org.firstinspires.ftc.teamcode.followers

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.util.NanoClock
import org.hermitsocialclub.drive.config.DriveConstants.MAX_TELE_VELO

class HolonomicPIDVAFollowerAccessible @JvmOverloads constructor(
    axialCoeffs: PIDCoefficients,
    lateralCoeffs: PIDCoefficients,
    headingCoeffs: PIDCoefficients,
    admissibleError: Pose2d = Pose2d(),
    timeout: Double = 0.0,
    clock: NanoClock = NanoClock.system()
) : TrajectoryFollower(admissibleError, timeout, clock) {
    private val axialController = PIDFController(axialCoeffs)
    private val lateralController = PIDFController(lateralCoeffs)
    private val headingController = PIDFController(headingCoeffs)

    override var lastError: Pose2d = Pose2d()

    init {
        headingController.setInputBounds(-Math.PI, Math.PI)
    }

    override fun followTrajectory(trajectory: Trajectory) {
        axialController.reset()
        lateralController.reset()
        headingController.reset()

        super.followTrajectory(trajectory)
    }

    override fun internalUpdate(currentPose: Pose2d, currentRobotVel: Pose2d?): DriveSignal {
        val t = elapsedTime()

        val targetPose = trajectory[t]
        val targetVel = trajectory.velocity(t)
        val targetAccel = trajectory.acceleration(t)

        val targetRobotVel = Kinematics.fieldToRobotVelocity(targetPose, targetVel)
        val targetRobotAccel = Kinematics.fieldToRobotAcceleration(targetPose, targetVel, targetAccel)

        val poseError = Kinematics.calculatePoseError(targetPose, currentPose)

        // you can pass the error directly to PIDFController by setting setpoint = error and measurement = 0
        axialController.targetPosition = poseError.x
        lateralController.targetPosition = poseError.y
        headingController.targetPosition = poseError.heading

        axialController.targetVelocity = targetRobotVel.x
        lateralController.targetVelocity = targetRobotVel.y
        headingController.targetVelocity = targetRobotVel.heading

        // note: feedforward is processed at the wheel level
        val axialCorrection = axialController.update(0.0, currentRobotVel?.x)
        val lateralCorrection = lateralController.update(0.0, currentRobotVel?.y)
        val headingCorrection = headingController.update(0.0, currentRobotVel?.heading)

        val correctedVelocity = targetRobotVel + Pose2d(
            axialCorrection,
            lateralCorrection,
            headingCorrection
        )

        lastError = poseError

        return DriveSignal(correctedVelocity, targetRobotAccel)
    }
    fun internalUpdateTeleop(currentPose: Pose2d, currentRobotVel: Pose2d?, drivePose: Pose2d): DriveSignal {
        val t = elapsedTime()

        val targetPose = Pose2d(
            currentPose.x + drivePose.x / 10,
            currentPose.y + drivePose.y / 10, currentPose.heading
        )
        val targetVel = drivePose
        val targetAccel = drivePose.minus(currentRobotVel!!)

        val targetRobotVel = Kinematics.fieldToRobotVelocity(targetPose, targetVel)
        val targetRobotAccel = Kinematics.fieldToRobotAcceleration(targetPose, targetVel, targetAccel)

        val hypotTargetVel = Math.hypot(targetRobotVel.x, targetRobotVel.y)
        val hypotMaxVel = Math.signum(hypotTargetVel) * Math.min(Math.abs(hypotTargetVel), MAX_TELE_VELO)
        val velMod = hypotMaxVel / hypotTargetVel
        val adjustedTargetRobotVel = Pose2d(targetRobotVel.x * velMod, targetRobotVel.y * velMod)

        val poseError = Kinematics.calculatePoseError(targetPose, currentPose)

        // you can pass the error directly to PIDFController by setting setpoint = error and measurement = 0
        axialController.targetPosition = poseError.x
        lateralController.targetPosition = poseError.y
        headingController.targetPosition = poseError.heading

        axialController.targetVelocity = adjustedTargetRobotVel.x
        lateralController.targetVelocity = adjustedTargetRobotVel.y
        headingController.targetVelocity = adjustedTargetRobotVel.heading

        // note: feedforward is processed at the wheel level
        val axialCorrection = axialController.update(0.0, currentRobotVel?.x)
        val lateralCorrection = lateralController.update(0.0, currentRobotVel?.y)
        val headingCorrection = headingController.update(0.0, currentRobotVel?.heading)

        val correctedVelocity = targetRobotVel + Pose2d(
            axialCorrection,
            lateralCorrection,
            headingCorrection
        )

        lastError = poseError

        return DriveSignal(correctedVelocity, targetRobotAccel)
    }
}
