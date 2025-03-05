package frc.robot.commands

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.apriltag.AprilTagPoseEstimate
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj2.command.Command
import frc.lib.rotation2dFromDeg
import frc.lib.swerve.SwerveDriveBase
import frc.lib.swerve.SwerveDriveConstants.DrivetrainConsts.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
import frc.lib.swerve.SwerveDriveConstants.DrivetrainConsts.MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND
import frc.lib.swerve.SwerveDriveConstants.DrivetrainConsts.thetaPIDController
import frc.lib.swerve.SwerveDriveConstants.DrivetrainConsts.xPIDController
import frc.lib.swerve.SwerveDriveConstants.DrivetrainConsts.yPIDController
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.math.abs
import kotlin.math.cos

class AlignToAprilTagCommand(val drive:SwerveDriveBase): Command() {
    lateinit var bestTarget: PhotonTrackedTarget

    var fidID = -1
    var targetDelta = Pose2d()
    val field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)
    var targetloc = Pose2d()

    init {
        addRequirements(drive)
    }

    override fun initialize() {
        if (!drive.camera!!.frame.hasTargets()) {
            end(false)
        } else {
            bestTarget = drive.camera!!.frame.bestTarget
            fidID = bestTarget.fiducialId
            val onright = bestTarget.bestCameraToTarget.y >= 0
            targetloc = field.getTagPose(fidID).get().toPose2d()
            //replace 0.5 with real target distance
            targetloc = Pose2d(targetloc.x + targetloc.rotation.sin * (if (onright) -1 else 1) * 0.5,targetloc.y - targetloc.rotation.cos * (if (onright) -1 else 1) * 0.5, targetloc.rotation)
        }
    }

    override fun execute() {
        if (fidID != -1) {
            targetDelta = Pose2d(targetloc.x - drive.getPose().x, targetloc.y - drive.getPose().y, (targetloc.rotation.degrees + 180 - drive.getPose().rotation.degrees).rotation2dFromDeg())
            println(targetDelta.x)
            drive.setModuleStates(
                drive.constructModuleStatesFromChassisSpeeds(
                    -xPIDController.calculate(targetDelta.x, 0.0),
                    -yPIDController.calculate(targetDelta.y, 0.0),
                    -thetaPIDController.calculate(targetDelta.rotation.degrees,
                        0.0
                    ) * MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND / 2,
                    true
                )

            )



            println(xPIDController.calculate(targetDelta.y, 0.0))
        }




    }

    override fun isFinished(): Boolean {
        if (fidID == -1){
            return true
        }
        else {
            //bestTarget.yaw < 5.0 &&
            return (abs(targetDelta.x) < 0.2 && abs(targetDelta.y) < 0.2 && abs(targetDelta.rotation.degrees) < 1)
        }
    }
}