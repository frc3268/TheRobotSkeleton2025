package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.lib.swerve.SwerveDriveBase
import frc.robot.Constants
import frc.robot.algaeintake.AlgaeIntakeSubsystem
import frc.robot.climber.ClimberSubsystem
import frc.robot.coralintake.CoralIntakeSubsystem
import frc.robot.elevator.ElevatorSubsystem

/** Setup shuffleboard buttons
 *
 * TODO: Put controller stuff in here */

/** High level routines / commands consisting of lower level commands */
object Routines {

    //intakes new coral from the source
    //raises coral arm, intakes coral, does NOT lower
    fun takeCoral(coralIntake: CoralIntakeSubsystem, elevator: ElevatorSubsystem): Command = SequentialCommandGroup(
        coralIntake.raiseToIntake().alongWith(elevator.setToPosition(-8.0)).andThen(coralIntake.intake()).andThen(coralIntake.lower()).andThen(elevator.setToPosition(Constants.Levels.LEVEL0.lvl))
    )

    // scores coral on reef at level
    //raise elevator, raise coral arm, run coral outake, lower coral arm, lower elevator
    fun placeCoralAtLevel(level: Double, elevator: ElevatorSubsystem, coralIntake: CoralIntakeSubsystem): Command =
        SequentialCommandGroup(
            elevator.setToPosition(level).andThen(
                coralIntake.raiseToScore()
            )
//            .andThen(coralIntake.outtake() ).andThen(coralIntake.lower())
//            .andThen(elevator.setToPosition(Constants.Levels.LEVEL0.lvl)))
        )

    //takes algae from reef at level
    //raise elevator, moves coral arm out of way, raises algae arm, runs algae flywheels, lowers elevator
    fun takeAlgaeAtLevel(level: Double, elevator: ElevatorSubsystem, algaeIntake: AlgaeIntakeSubsystem, coralIntake: CoralIntakeSubsystem): Command = SequentialCommandGroup(
        elevator.runOnce { elevator.setToPosition(level) },
        coralIntake.runOnce { coralIntake.raiseToIntake() },
        algaeIntake.runOnce { algaeIntake.raise() }
            .andThen({ algaeIntake.intake() }),
        //coralIntake.runOnce { coralIntake.lower() }, //this would hit the algae
        elevator.runOnce { elevator.setToPosition(Constants.Levels.LEVEL0.lvl) }
    )

    //stops everything, doesn't lower
    fun stopAll(elevator: ElevatorSubsystem, algaeIntake: AlgaeIntakeSubsystem, coralIntake: CoralIntakeSubsystem, climberSubsystem: ClimberSubsystem): Command = SequentialCommandGroup(
        elevator.runOnce { elevator.stop() },
        algaeIntake.runOnce { algaeIntake.stopAll() },
        coralIntake.runOnce { coralIntake.stop() },
        climberSubsystem.run { climberSubsystem.stop() }
    )

    fun inchForward(drive: SwerveDriveBase) = SwerveJoystickDrive(drive, {0.1}, {0.0}, {0.0}, {false}).withTimeout(0.5)

    fun inchBack(drive: SwerveDriveBase) = SwerveJoystickDrive(drive, {-0.1}, {0.0}, {0.0}, {false}).withTimeout(0.5)

    fun inchLeft(drive: SwerveDriveBase) = SwerveJoystickDrive(drive, {0.0}, {-0.1}, {0.0}, {false}).withTimeout(0.5)

    fun inchRight(drive: SwerveDriveBase) = SwerveJoystickDrive(drive, {0.0}, {0.1}, {0.0}, {false}).withTimeout(0.5)

}