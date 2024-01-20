package frc.lib.utils

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.lib.basics.SwerveDriveBase

class TrajectoryOrchestrator {

    //take in a trajectory and display it on the tab
    
    //update display when trajectory is changed

    companion object {
        fun buildSwerveTrajectory(startPose:Pose2d, endPose:Pose2d, points:MutableList<Translation2d>, drive:SwerveDriveBase): SequentialCommandGroup {
            val scg:SequentialCommandGroup = SequentialCommandGroup()
            val relativeTo:Pose2d = drive.getPose()
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
    }


    //2 methods to add trajectory: one for swerve and one for differential
    
    //cancel trajectory when updated (?)
}