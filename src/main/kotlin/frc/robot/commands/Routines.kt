package frc.robot.commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.algaeintake.AlgaeIntakeSubsystem
import frc.robot.climber.ClimberSubsystem
import frc.robot.coralintake.CoralIntakeSubsystem
import frc.robot.elevator.ElevatorSubsystem

enum class Levels(val lvl: Double) {
    LEVEL0(0.0),
    LEVEL1(0.0),
    LEVEL2(0.0),
    LEVEL3(0.0),
    LEVEL4(0.0)
}

// Setup shuffleboard
fun initDashboard(elevator: ElevatorSubsystem, algaeIntake: AlgaeIntakeSubsystem, coralIntake: CoralIntakeSubsystem, climberSubsystem: ClimberSubsystem) {
    // Edit this, none of these work right now
    SmartDashboard.putData("Reset elevator position", Routines.resetElevator(elevator));
    SmartDashboard.putData("STOP ALL", Routines.stopAll(elevator, algaeIntake, coralIntake, climberSubsystem));
}

object Routines {

    // resetElevator should be called after this

    // Removes algae from the reef
    fun takeAlgae(level: Double, elevator: ElevatorSubsystem, algaeIntake: AlgaeIntakeSubsystem): Command = SequentialCommandGroup(
        elevator.runOnce { elevator.setToPosition(level) },
        algaeIntake.runOnce {
            algaeIntake.raise()
            algaeIntake.intake()
        }
    )

    fun takeCoral(coralIntake: CoralIntakeSubsystem): Command = SequentialCommandGroup(
        coralIntake.runOnce { coralIntake.raiseToIntake() }.andThen(
            coralIntake.intake()
        )
    )

    // resetElevator should be called after this
    fun placeCoralAtLevel(level: Double, elevator: ElevatorSubsystem, coralIntake: CoralIntakeSubsystem): Command = SequentialCommandGroup(
        elevator.runOnce { elevator.setToPosition(level) },
        // TODO: This looks ugly, should be rewritten
        coralIntake.runOnce { coralIntake.raiseToScore() }.andThen(
            { coralIntake.outtake() }
        ).andThen( { coralIntake.lower() } )
    )

    fun stopAll(elevator: ElevatorSubsystem, algaeIntake: AlgaeIntakeSubsystem, coralIntake: CoralIntakeSubsystem, climberSubsystem: ClimberSubsystem): Command = SequentialCommandGroup(
        elevator.runOnce { elevator.stop() },
        algaeIntake.runOnce { algaeIntake.stopAll() },
        coralIntake.runOnce { coralIntake.stop() }
    )

    fun resetElevator(elevator: ElevatorSubsystem): Command = RunCommand(
        { elevator.setToPosition(0.0) }
    )
}