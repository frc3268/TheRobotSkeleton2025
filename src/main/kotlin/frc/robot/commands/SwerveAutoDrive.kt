package frc.robot.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.Command
import frc.lib.FieldPositions.obstacles
import frc.lib.Obstacle
import frc.lib.rotation2dFromDeg
import frc.lib.rotation2dFromRad
import frc.lib.swerve.SwerveDriveBase
import frc.lib.swerve.SwerveDriveConstants
import frc.lib.swerve.SwerveDriveConstants.DrivetrainConsts.MAX_SPEED_METERS_PER_SECOND
import frc.robot.Constants
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import kotlin.math.*

/*
SwerveAutoDrive, command for swerve driving which fuses setpoint following and remote control
allows the user to take manual control of the joysticks to make adjustments while also sending the robot to the setpoint
 */
class SwerveAutoDrive(
    private val setpoint: Supplier<Pose2d>,
    private val drive: SwerveDriveBase
): Command() {
    private var index = 0
    private var points = mutableListOf<Pose2d>()
    var next = Pose2d()
    private var tolerance: Pose2d = Pose2d(0.1,0.1, 10.0.rotation2dFromDeg())

    init {
        addRequirements(drive)
    }

    override fun initialize() {
        index = 0
        points = pathfind(drive.getPose(), setpoint.get())
        points.add(0, drive.getPose())
        drive.field.getObject("points").setPoses(points)
        next = points[0]
    }

    override fun execute() {
        /*collect speeds based on which controls are used*/

        val speeds = Pose2d(
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
                    TrapezoidProfile.State(next.rotation.degrees, 0.0)
                ) * SwerveDriveConstants.DrivetrainConsts.MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND).rotation2dFromDeg(),

            )

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
            abs(drive.getPose().x - next.x) < tolerance.x &&
            abs(drive.getPose().y - next.y) < tolerance.y &&
            abs(drive.getPose().rotation.minus(next.rotation).degrees) < tolerance.rotation.degrees
        ) {
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
        //this whole thing could be faster...
        val sortedObstacles = obstacles.sortedBy { it.location.translation.getDistance(from.translation) }
        val m: Double = (to.y - from.y) / (to.x - from.x)
        val b: Double = -m * to.x + to.y
        for (obstacle in sortedObstacles) {
            val ntwo = b - obstacle.location.y
            val obx = obstacle.location.x
            val btwo = -obx * 2 + ntwo * 2 * m
            val a = m.pow(2) + 1
            val c = ntwo.pow(2) + obx.pow(2) - obstacle.radiusMeters.pow(2)
            val det = btwo.pow(2) - 4 * a * c
            if (det >= 0) {
                val intersectiona: Pose2d = if (from.x > obstacle.location.x) Pose2d(
                    ((-btwo + sqrt(det)) / (2 * a)), (m * ((-btwo + sqrt(det)) / (2 * a)) + b), from.rotation
                ) else Pose2d(
                    ((-btwo - sqrt(det)) / (2 * a)), (m * ((-btwo - sqrt(det)) / (2 * a)) + b), from.rotation
                )
                if (intersectiona.x in from.x..to.x || intersectiona.x in to.x..from.x) {

                        val intersectionb = if (from.x < obstacle.location.x) Pose2d(
                            ((-btwo + sqrt(det)) / (2 * a)), (m * ((-btwo + sqrt(det)) / (2 * a)) + b), from.rotation
                        ) else Pose2d(
                            ((-btwo - sqrt(det)) / (2 * a)), (m * ((-btwo - sqrt(det)) / (2 * a)) + b), from.rotation
                        )

                    val linmidpoint = Pose2d(
                        (intersectionb.x + intersectiona.x) / 2,
                        (intersectionb.y + intersectiona.y) / 2,
                        0.0.rotation2dFromDeg()
                    )

                    val dist = obstacle.radiusMeters - abs(linmidpoint.translation.getDistance(obstacle.location.translation))

                    var rates = Pose2d( (dist) * (((linmidpoint.x - obstacle.location.x) / abs((linmidpoint.x - obstacle.location.x))) * linmidpoint.x * (obstacle.radiusMeters/ sqrt(linmidpoint.x + linmidpoint.y)))/obstacle.radiusMeters ,
                        (dist) * (((linmidpoint.y - obstacle.location.y) / abs((linmidpoint.y - obstacle.location.y))) * linmidpoint.y * (obstacle.radiusMeters/ sqrt(linmidpoint.x + linmidpoint.y) )) / obstacle.radiusMeters,
                            from.rotation

                    )

                    var waypoint =
                        Pose2d(
                            linmidpoint.translation.plus(rates.translation),
                            from.rotation
                        )

                    if(waypoint.x !in 0.0..16.4846 || waypoint.y !in 0.0..8.1026) {
                        rates = Pose2d( (dist) * -(((linmidpoint.x - obstacle.location.x) / abs((linmidpoint.x - obstacle.location.x))) * linmidpoint.x * (obstacle.radiusMeters/ sqrt(linmidpoint.x + linmidpoint.y)))/obstacle.radiusMeters ,
                            (dist) * -(((linmidpoint.y - obstacle.location.y) / abs((linmidpoint.y - obstacle.location.y))) * linmidpoint.y * (obstacle.radiusMeters/ sqrt(linmidpoint.x + linmidpoint.y) )) / obstacle.radiusMeters,
                            from.rotation

                        )
                        waypoint =
                            Pose2d(
                                linmidpoint.translation.plus(rates.translation),
                                from.rotation
                            )
                    }


                    return mutableListOf(pathfind(from, waypoint), pathfind(waypoint, to)).flatten().toMutableList()
                }
            }
        }
            return mutableListOf(to)
    }
}