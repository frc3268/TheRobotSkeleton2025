package frc.robot.commands

import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj2.command.Command
import frc.lib.swerve.SwerveDriveBase
import frc.lib.swerve.SwerveDriveConstants.DrivetrainConsts.MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND
import frc.lib.swerve.SwerveDriveConstants.DrivetrainConsts.thetaPIDController
import frc.lib.swerve.SwerveDriveConstants.DrivetrainConsts.xPIDController
import frc.lib.swerve.SwerveDriveConstants.DrivetrainConsts.yPIDController
import org.photonvision.targeting.PhotonTrackedTarget

class AlignToAprilTagCommand(val drive:SwerveDriveBase): Command() {
    lateinit var bestTarget: PhotonTrackedTarget

    init {
        addRequirements(drive)
    }

    override fun initialize() {
        if(!drive.camera!!.frame.hasTargets()){
            end(false)
        } else{
            bestTarget = drive.camera!!.frame.bestTarget
        }
    }

    override fun execute() {
        if(drive.camera!!.frame.hasTargets()){
            bestTarget = drive.camera!!.frame.bestTarget
        }
        drive.setModuleStates(
            //these are negative cause otherwise robot would crash out trying to go backwards
            //fieldoriented is false for a similar reason
            drive.constructModuleStatesFromChassisSpeeds(
                //may want to change setpoints
                xPIDController.calculate(bestTarget.bestCameraToTarget.x, 0.3) * 0.5,
                yPIDController.calculate(bestTarget.bestCameraToTarget.y, 0.3) * 0.5,
                thetaPIDController.calculate(bestTarget.getYaw(), 0.0) * MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND,
                false
            )
        )

    }

    override fun isFinished(): Boolean {
        return bestTarget.yaw < 5.0 && bestTarget.bestCameraToTarget.translation.getDistance(
            Translation3d(0.0,0.0,0.0)
        ) < 0.1
    }
}