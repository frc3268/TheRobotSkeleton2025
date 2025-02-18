package frc.robot.commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.algaeintake.AlgaeIntakeSubsystem
import frc.robot.coralintake.CoralIntakeSubsystem
import frc.robot.elevator.ElevatorSubsystem

// Setup shuffleboard
fun initDashboard(elevator: ElevatorSubsystem, algaeIntake: AlgaeIntakeSubsystem, coralIntake: CoralIntakeSubsystem) {
    // Edit this, none of these work right now
    SmartDashboard.putData("Reset elevator position", Routines.resetElevator(elevator));
    SmartDashboard.putData("Take Algae: l1", Routines.takeAlgae(0.0, elevator, algaeIntake));
    SmartDashboard.putData("Take Algae: l2", Routines.takeAlgae(0.0, elevator, algaeIntake));
    SmartDashboard.putData("Take Algae: l3", Routines.takeAlgae(0.0, elevator, algaeIntake));
    SmartDashboard.putData("Take Algae: l4", Routines.takeAlgae(0.0, elevator, algaeIntake));

    // TODO: Replace this with a higher-level command
    SmartDashboard.putData("Take Coral", Routines.takeCoral(coralIntake));

    SmartDashboard.putData("Place Coral: l1", Routines.placeCoralAtLevel(0.0, elevator, coralIntake));
    SmartDashboard.putData("Place Coral: l2", Routines.placeCoralAtLevel(0.0, elevator, coralIntake));
    SmartDashboard.putData("Place Coral: l3", Routines.placeCoralAtLevel(0.0, elevator, coralIntake));
    SmartDashboard.putData("Place Coral: l4", Routines.placeCoralAtLevel(0.0, elevator, coralIntake));

    SmartDashboard.putData("STOP ALL", Routines.stopAll(elevator, algaeIntake, coralIntake));
}

object Routines {
<<<<<<< Updated upstream

    // resetElevator should be called after this
=======
    // Removes algae from the coral tree
>>>>>>> Stashed changes
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

    fun stopAll(elevator: ElevatorSubsystem, algaeIntake: AlgaeIntakeSubsystem, coralIntake: CoralIntakeSubsystem): Command = SequentialCommandGroup(
        elevator.runOnce { elevator.stop() },
        algaeIntake.runOnce { algaeIntake.stopAll() },
        coralIntake.runOnce { coralIntake.stop() }
    )

    fun resetElevator(elevator: ElevatorSubsystem): Command = RunCommand(
        { elevator.setToPosition(0.0) }
    )
}