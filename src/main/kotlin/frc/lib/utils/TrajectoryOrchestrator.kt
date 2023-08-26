package frc.lib.utils

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.trajectory.TrajectoryConfig
import edu.wpi.first.math.trajectory.TrajectoryGenerator
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand
import frc.lib.basics.SwerveDriveBase
import frc.lib.constants.SwerveDriveConstants

class TrajectoryOrchestrator {
    private val tab: ShuffleboardTab = Shuffleboard.getTab("Drivetrain")
    private var trajectory: Trajectory = Trajectory()
    val startAtRobotPosition : Pose2d = Pose2d(0.0,0.0, Rotation2d.fromDegrees(0.0))

    //take in a trajectory and display it on the tab
    
    //update display when trajectory is changed
    
    fun buildSwerveTrajectory(startPose:Pose2d, endPose:Pose2d, points:MutableList<Translation2d>, drive:SwerveDriveBase): SequentialCommandGroup {
        val trajectoryConfig:TrajectoryConfig = TrajectoryConfig(
            SwerveDriveConstants.DrivetrainConsts.MAX_SPEED_METERS_PER_SECOND,
            SwerveDriveConstants.DrivetrainConsts.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
        ).setKinematics(drive.kinematics)

        val trajectory:Trajectory = TrajectoryGenerator.generateTrajectory(
            startPose,
            points,
            endPose,
            trajectoryConfig
        )

        val trajectoryCommand = SwerveControllerCommand(
            trajectory,
            drive::getPose,
            drive.kinematics,
            SwerveDriveConstants.DrivetrainConsts.xPIDController,
            SwerveDriveConstants.DrivetrainConsts.yPIDController,
            SwerveDriveConstants.DrivetrainConsts.thetaPIDController,
            drive::setModuleStates,
            drive
        )

        return SequentialCommandGroup(
            trajectoryCommand,
            InstantCommand({drive.stop()})
        )
    }

    //2 methods to add trajectory: one for swerve and one for differential
    
    //cancel trajectory when updated (?)
}