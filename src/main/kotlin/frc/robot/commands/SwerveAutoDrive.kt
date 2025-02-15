package frc.robot.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.lib.*
import frc.lib.swerve.SwerveDriveBase
import frc.lib.swerve.SwerveDriveConstants
import frc.lib.swerve.SwerveDriveConstants.DrivetrainConsts.MAX_SPEED_METERS_PER_SECOND
import frc.robot.Constants
import kotlinx.serialization.ExperimentalSerializationApi
import kotlinx.serialization.Serializable
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.decodeFromStream
import java.io.File
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import kotlin.math.*

@Serializable
data class gridFile(val grid: Array<Array<Boolean>>)


/*
SwerveAutoDrive, command for swerve driving which fuses setpoint following and remote control
allows the user to take manual control of the joysticks to make adjustments while also sending the robot to the setpoint
 */
class SwerveAutoDrive(
    private val setpoint: Supplier<Pose2d>,
    private val drive: SwerveDriveBase,
    private val grid:Array<Array<Boolean>>
): Command() {
    private var index = 0
    private var points = mutableListOf<Pose2d>()

    private var start = 0.0
    var next = Pose2d()
    var startPose = drive.getPose()
    private var tolerance: Pose2d = Pose2d(0.1, 0.1, 10.0.rotation2dFromDeg())
    val to = setpoint.get()

    init {
        addRequirements(drive)
    }

    override fun initialize() {
        index = 0
        points = pathfind(drive.getPose(),  to)
        points.add(0, drive.getPose())
        drive.field.getObject("points").setPoses(points)
        next = points[0]
        System.out.println("began")
    }

    override fun execute() {
        /*collect speeds based on which controls are used*/
        val speeds =
            if (Constants.mode == Constants.States.SIM){
                Pose2d(
                    SwerveDriveConstants.DrivetrainConsts.xPIDController.calculate(
                        drive.getPose().x,
                        TrapezoidProfile.State(next.x, 0.0)
                    ) * MAX_SPEED_METERS_PER_SECOND,
                    SwerveDriveConstants.DrivetrainConsts.yPIDController.calculate(
                        drive.getPose().y,
                        TrapezoidProfile.State(next.y, 0.0)
                    ) * MAX_SPEED_METERS_PER_SECOND,
                    (SwerveDriveConstants.DrivetrainConsts.thetaPIDController.calculate(
                        drive.getPose().rotation.degrees,
                        TrapezoidProfile.State(to.rotation.degrees, 0.0)
                    ) * SwerveDriveConstants.DrivetrainConsts.MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND).rotation2dFromDeg(),

                    )
            }
        else {
                Pose2d(
                    -SwerveDriveConstants.DrivetrainConsts.xPIDController.calculate(
                        drive.getPose().x,
                        TrapezoidProfile.State(next.x, 0.0)
                    ) * MAX_SPEED_METERS_PER_SECOND,
                    -SwerveDriveConstants.DrivetrainConsts.yPIDController.calculate(
                        drive.getPose().y,
                        TrapezoidProfile.State(next.y, 0.0)
                    ) * MAX_SPEED_METERS_PER_SECOND,
                    (SwerveDriveConstants.DrivetrainConsts.thetaPIDController.calculate(
                        drive.getPose().rotation.degrees,
                        TrapezoidProfile.State(to.rotation.degrees, 0.0)
                    ) * SwerveDriveConstants.DrivetrainConsts.MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND).rotation2dFromDeg(),

                    )
            }

        /* Drive */
        drive.setModuleStates(
            drive.constructModuleStatesFromChassisSpeeds(
                speeds.x,
                speeds.y,
                speeds.rotation.degrees,
                true
            )
        )

    }

    override fun isFinished(): Boolean {
        if (
            (abs(drive.getPose().x - next.x) < 0.5 &&
            abs(drive.getPose().y - next.y) < 0.5) && index < points.size - 1 ||
            (abs(drive.getPose().x - next.x) < tolerance.x &&
                    abs(drive.getPose().y - next.y) < tolerance.y &&
                    (abs(drive.getPose().rotation.minus(next.rotation).degrees) < tolerance.rotation.degrees) ) && index >= points.size - 1){
            if (index >= points.size - 1) {
                return true
            }
            index++
            next = points[index]
        }
        return false
    }

    override fun end(interrupted: Boolean) {
    }

    private fun pathfind(from: Pose2d, to: Pose2d): MutableList<Pose2d> {
        val path =
            smoothPath(
                construct_path(
                a_star(
                    Pair((from.x / .3).toInt(), (from.y / .3).toInt()),
                    Pair((to.x / .3).toInt(), (to.y / .3).toInt()),
                    grid
                )
            ))
        path.add(to)
        return path
    }
}