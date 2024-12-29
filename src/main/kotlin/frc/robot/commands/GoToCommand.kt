package frc.robot.commands

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.lib.FieldLocation
import frc.lib.FieldPositions
import frc.lib.FieldPositions.obstacles
import frc.lib.rotation2dFromDeg
import frc.lib.swerve.SwerveDriveBase
import frc.lib.swerve.SwerveDriveConstants
import java.util.function.DoubleSupplier
import kotlin.math.pow
import kotlin.math.sqrt

class GoToCommand(
    private val setpoint: Pose2d,
    private val drive: SwerveDriveBase,
    private val translationX: DoubleSupplier,
    private val translationY: DoubleSupplier,
    private val rotation: DoubleSupplier)
    :    Command() {

    private var midpoint: Pose2d = Pose2d()

    override fun initialize() {
        addRequirements(drive)
        val pose = drive.getPose()
        val m:Double = (setpoint.y-pose.y) / (setpoint.x-pose.x)
        val b:Double = -setpoint.y/setpoint.x + setpoint.x

        for(obstacle in obstacles){
            val ntwo = b - obstacle.location.y
            val obx = obstacle.location.x
            val btwo = -obx * 2 + ntwo * 2
            val a = m.pow(2)
            val c = ntwo.pow(2) + obx.pow(2) - obstacle.radiusMeters.pow(2)
            val det = btwo.pow(2) - 4 * a * c
            if(det >= 0){
                val intersection:Pose2d =  if(pose.x > obstacle.location.x) Pose2d(
                    ((-btwo + sqrt(det))/2*a), (m*((-btwo + sqrt(det))/2*a) + b), pose.rotation
                ) else Pose2d(
                    ((-btwo - sqrt(det))/2*a), (m*((-btwo + sqrt(det))/2*a) + b), pose.rotation)

                midpoint = Pose2d(intersection.x+SwerveDriveConstants.DrivetrainConsts.TRACK_WIDTH_METERS,-1/m*(intersection.x + SwerveDriveConstants.DrivetrainConsts.TRACK_WIDTH_METERS) + b, pose.rotation)
                end(interrupted = true)
            }
        }
        end(false)
    }

    override fun end(interrupted: Boolean) {
        if(interrupted) {
            SequentialCommandGroup(GoToCommand(midpoint,
                drive,
                translationX, translationY, rotation),GoToCommand(setpoint,
                drive,
                translationX, translationY, rotation)).schedule()
        } else{
            SwerveAutoDrive(setpoint,Pose2d(0.1, 0.1, 10.0.rotation2dFromDeg()),drive,translationX, translationY,rotation).schedule()
        }
    }

}