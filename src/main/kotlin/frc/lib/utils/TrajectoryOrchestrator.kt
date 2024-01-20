package frc.lib.utils

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.lib.basics.SwerveDriveBase

class TrajectoryOrchestrator {

    //take in a trajectory and display it on the tab
    
    //update display when trajectory is changed

    init{

        
    }


    fun buildSwerveTrajectory(startPose:Pose2d, endPose:Pose2d, points:MutableList<Translation2d>, drive:SwerveDriveBase): SequentialCommandGroup {
        val scg:SequentialCommandGroup = SequentialCommandGroup()
        val relativeTo:Pose2d = drive.getPose()
        scg.addCommands(drive.robotPoseToBCommand(makePoseRelative(startPose, relativeTo)))
        for(point in points){
            scg.addCommands(drive.robotPoseToBCommand(makePoseRelative(Pose2d(point, drive.getYaw()), relativeTo)))
        }
        scg.addCommands(drive.robotPoseToBCommand(makePoseRelative(endPose, relativeTo)))
        scg.addCommands(InstantCommand({drive.stop()}))
        scg.addRequirements(drive)
        return scg
    }
    
    fun makePoseRelative(pose:Pose2d, relativeTo: Pose2d): Pose2d{
        return Pose2d(
            Translation2d(
                pose.x + relativeTo.x,
                pose.y + relativeTo.y,
            ),
            pose.rotation
        )
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