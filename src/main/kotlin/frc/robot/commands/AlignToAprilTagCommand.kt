package frc.robot.commands

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.apriltag.AprilTagPoseEstimate
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj2.command.Command
import frc.lib.swerve.SwerveDriveBase
import frc.lib.swerve.SwerveDriveConstants.DrivetrainConsts.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
import frc.lib.swerve.SwerveDriveConstants.DrivetrainConsts.MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND
import frc.lib.swerve.SwerveDriveConstants.DrivetrainConsts.thetaPIDController
import frc.lib.swerve.SwerveDriveConstants.DrivetrainConsts.xPIDController
import frc.lib.swerve.SwerveDriveConstants.DrivetrainConsts.yPIDController
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.math.abs

class AlignToAprilTagCommand(val drive:SwerveDriveBase): Command() {
    lateinit var bestTarget: PhotonTrackedTarget

    init {
        addRequirements(drive)
    }

    override fun initialize() {
        if (!drive.camera!!.frame.hasTargets()) {
            end(false)
        } else {
            bestTarget = drive.camera!!.frame.bestTarget
        }
    }

    override fun execute() {
        if (drive.camera!!.frame.hasTargets()) {
            bestTarget = drive.camera!!.frame.bestTarget
            drive.setModuleStates(
                //these are negative cause otherwise robot would crash out trying to go backwards
                //fieldoriented is false for a similar reason
                drive.constructModuleStatesFromChassisSpeeds(
                    //may want to change setpoints
                    // 0.0, 0.0,
                    xPIDController.calculate(bestTarget.bestCameraToTarget.x, 0.0),
                    -yPIDController.calculate(bestTarget.bestCameraToTarget.y, 0.0),
                    thetaPIDController.calculate(bestTarget.getYaw(), 0.0) * MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND/2,
                    false
                )
            )
        }




    }

    override fun isFinished(): Boolean {
        if (!drive.camera!!.frame.hasTargets()){
            return true
        }
        else {
            //bestTarget.yaw < 5.0 &&
            return (abs(bestTarget.bestCameraToTarget.x) < 0.05 && abs(bestTarget.bestCameraToTarget.y) < 0.05 && abs(bestTarget.getYaw()) > 1)
        }
    }
}