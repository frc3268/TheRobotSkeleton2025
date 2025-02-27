package frc.robot.commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Constants
import frc.robot.algaeintake.AlgaeIntakeSubsystem
import frc.robot.climber.ClimberSubsystem
import frc.robot.coralintake.CoralIntakeSubsystem
import frc.robot.elevator.ElevatorSubsystem

// Setup shuffleboard
fun initDashboard(elevator: ElevatorSubsystem, algaeIntake: AlgaeIntakeSubsystem, coralIntake: CoralIntakeSubsystem, climberSubsystem: ClimberSubsystem) {
    SmartDashboard.putData("STOP ALL", Routines.stopAll(elevator, algaeIntake, coralIntake, climberSubsystem));
}

object Routines {

    // resetElevator should be called after this
    // Removes algae from the reef

    fun takeCoral(coralIntake: CoralIntakeSubsystem): Command = SequentialCommandGroup(
        coralIntake.runOnce { coralIntake.raiseToIntake() }.andThen(
            coralIntake.intake()
        )
    )

    // resetElevator should be called after this
    fun placeCoralAtLevel(level: Double, elevator: ElevatorSubsystem, coralIntake: CoralIntakeSubsystem): Command = SequentialCommandGroup(
        elevator.runOnce { elevator.setToPosition(level) },
        coralIntake.runOnce { coralIntake.raiseToScore() }
            .andThen({ coralIntake.outtake() })
            .andThen({ coralIntake.lower() } ),
        elevator.runOnce { elevator.setToPosition(Constants.Levels.LEVEL0.lvl) }
    )

    fun takeAlgaeAtLevel(level: Double, elevator: ElevatorSubsystem, algaeIntake: AlgaeIntakeSubsystem, coralIntake: CoralIntakeSubsystem): Command = SequentialCommandGroup(
        elevator.runOnce { elevator.setToPosition(level) },
        coralIntake.runOnce { coralIntake.raiseToIntake() },
        algaeIntake.runOnce { algaeIntake.raise() }
            .andThen({ algaeIntake.intake() }),
        coralIntake.runOnce { coralIntake.lower() },
        elevator.runOnce { elevator.setToPosition(Constants.Levels.LEVEL0.lvl) }
    )
    fun stopAll(elevator: ElevatorSubsystem, algaeIntake: AlgaeIntakeSubsystem, coralIntake: CoralIntakeSubsystem, climberSubsystem: ClimberSubsystem): Command = SequentialCommandGroup(
        elevator.runOnce { elevator.stop() },
        algaeIntake.runOnce { algaeIntake.stopAll() },
        coralIntake.runOnce { coralIntake.stop() },
        climberSubsystem.run { climberSubsystem.stop() }
    )

    fun resetElevator(elevator: ElevatorSubsystem): Command = RunCommand(
        { elevator.setToPosition(0.0) }
    )
}