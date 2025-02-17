package frc.robot.commands

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.algaeintake.AlgaeIntakeSubsystem
import frc.robot.elevator.ElevatorSubsystem

// Setup shuffleboard
fun initDashboard(elevator: ElevatorSubsystem, algaeIntake: AlgaeIntakeSubsystem) {
    // Edit this, none of these work right now
    SmartDashboard.putData("Reset elevator position", Routines.resetElevator(elevator));
    SmartDashboard.putData("Take Algae: l1", Routines.takeAlgae(0.0, elevator, algaeIntake));
    SmartDashboard.putData("Take Algae: l2", Routines.takeAlgae(0.0, elevator, algaeIntake));
    SmartDashboard.putData("Take Algae: l3", Routines.takeAlgae(0.0, elevator, algaeIntake));
    SmartDashboard.putData("Take Algae: l4", Routines.takeAlgae(0.0, elevator, algaeIntake));
}

object Routines {
    fun takeAlgae(level: Double, elevator: ElevatorSubsystem, algaeIntake: AlgaeIntakeSubsystem): Command = SequentialCommandGroup(
        elevator.runOnce { elevator.setToPosition(level) },
        algaeIntake.runOnce {
            algaeIntake.raise()
            algaeIntake.intake()
        }
    )

    fun resetElevator(elevator: ElevatorSubsystem): Command = RunCommand(
        { elevator.setToPosition(0.0) }
    )

}