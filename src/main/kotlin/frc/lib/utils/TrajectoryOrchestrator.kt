package frc.lib.utils

import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab

class TrajectoryOrchestrator {
    private val tab: ShuffleboardTab = Shuffleboard.getTab("Drivetrain")
    private var trajectory: Trajectory = Trajectory()
    
    //take in a trajectory and display it on the tab
    
    //update display when trajectory is changed
    
    //2 methods to add trajectory: one for swerve and one for differential
    
    //cancel trajectory when updated (?)
}