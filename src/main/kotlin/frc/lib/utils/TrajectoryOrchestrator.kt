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
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand
import frc.lib.basics.SwerveDriveBase
import frc.lib.basics.SwerveModule
import frc.lib.constants.SwerveDriveConstants

class TrajectoryOrchestrator {

    //take in a trajectory and display it on the tab
    
    //update display when trajectory is changed

    init{

        
    }


    fun buildSwerveTrajectory(startPose:Pose2d, endPose:Pose2d, points:MutableList<Translation2d>, drive:SwerveDriveBase): SequentialCommandGroup {
        val scg:SequentialCommandGroup = SequentialCommandGroup()
        scg.addCommands(drive.robotPoseToBCommand(startPose))
        for(point in points){
            scg.addCommands(drive.robotPoseToBCommand(Pose2d(point, drive.getYaw())))
        }
        scg.addCommands(drive.robotPoseToBCommand(endPose))
        scg.addCommands(InstantCommand({drive.stop()}))
        scg.addRequirements(drive)
        return scg
    }
    
    fun beelineCommand(drive:SwerveDriveBase, to:Pose2d):SequentialCommandGroup{
        val scg:SequentialCommandGroup = SequentialCommandGroup()
        scg.addCommands(drive.robotPoseToBCommand(to))
        scg.addCommands(InstantCommand({drive.stop()}))
        scg.addRequirements(drive)
        return scg
    }


    //2 methods to add trajectory: one for swerve and one for differential
    
    //cancel trajectory when updated (?)
}