package frc.lib.utils

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import frc.lib.basics.SwerveDriveBase

class TrajectoryOrchestrator {
    private val tab: ShuffleboardTab = Shuffleboard.getTab("Drivetrain")
    private var trajectory: Trajectory = Trajectory()
    val startAtRobotPosition : Pose2d = Pose2d(0.0,0.0, Rotation2d.fromDegrees(0.0))

    //take in a trajectory and display it on the tab
    
    //update display when trajectory is changed
    
    fun startSwerveTrajectory(points: Array<Pose2d>, drive:SwerveDriveBase) {
        //voltageConstraint
        if(points[0] == startAtRobotPosition){
            //generate the trajectory at the robot's position
            return
        }
        //generate trajectory normally
    }

    //2 methods to add trajectory: one for swerve and one for differential
    
    //cancel trajectory when updated (?)
}