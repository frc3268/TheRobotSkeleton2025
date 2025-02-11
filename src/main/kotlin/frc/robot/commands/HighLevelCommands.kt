package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.algaeintake.AlgaeIntakeSubsystem
import frc.robot.elevator.ElevatorSubsystem

object HighLevelCommands {
    fun takeAlgae(level: Double, elevator: ElevatorSubsystem, algaeIntake: AlgaeIntakeSubsystem): Command = SequentialCommandGroup(
        elevator.runOnce { elevator.setToPosition(level) },
        algaeIntake.runOnce {
            algaeIntake.raise()
            algaeIntake.intake()
        }
    )
}