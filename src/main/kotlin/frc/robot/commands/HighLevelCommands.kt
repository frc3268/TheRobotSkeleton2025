package frc.robot.commands

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.algaeintake.AlgaeIntakeSubsystem
import frc.robot.elevator.ElevatorSubsystem

// Setup shuffleboard
fun initDashboard(elevator: ElevatorSubsystem, algaeIntake: AlgaeIntakeSubsystem) {
    // Edit this, none of these work right now
    SmartDashboard.putData("Take Alagae: l1", HighLevelCommands.takeAlgae(0.0, elevator, algaeIntake));
    SmartDashboard.putData("Take Alagae: l2", HighLevelCommands.takeAlgae(0.0, elevator, algaeIntake));
    SmartDashboard.putData("Take Alagae: l3", HighLevelCommands.takeAlgae(0.0, elevator, algaeIntake));
    SmartDashboard.putData("Take Alagae: l4", HighLevelCommands.takeAlgae(0.0, elevator, algaeIntake));

}
object HighLevelCommands {
    fun takeAlgae(level: Double, elevator: ElevatorSubsystem, algaeIntake: AlgaeIntakeSubsystem): Command = SequentialCommandGroup(
        elevator.runOnce { elevator.setToPosition(level) },
        algaeIntake.runOnce {
            algaeIntake.raise()
            algaeIntake.intake()
        }
    )
}