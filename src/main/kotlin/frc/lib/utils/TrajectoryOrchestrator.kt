package frc.lib.utils

import edu.wpi.first.math.geometry.*
import edu.wpi.first.wpilibj2.command.*
import frc.lib.basics.SwerveDriveBase

class TrajectoryOrchestrator {
    // TODO take in a trajectory and display it on the tab
    // TODO update display when trajectory is changed

    companion object {
        fun buildSwerveTrajectory(
            startPose: Pose2d,
            endPose: Pose2d,
            points: MutableList<Translation2d>,
            drive: SwerveDriveBase
        ): SequentialCommandGroup {
            // I refactored this to be functional-paradigmed and do the same thing as the earlier imperative code,
            // but I may hvae messed up. Please check the git history and roll back if it doesn't work.
            // -- Weiju
            val scg = SequentialCommandGroup(
                drive.moveToPoseCommand(startPose),
                *points.map { drive.moveToPoseCommand(Pose2d(it, drive.getYaw())) }.toTypedArray(),
                drive.moveToPoseCommand(endPose),
                InstantCommand({ drive.stop() })
            )
            scg.addRequirements(drive)
            return scg
        }

        //go directly to a point given by the argument "to"

    }


    //2 methods to add trajectory: one for swerve and one for differential
    
    //cancel trajectory when updated (?)
}