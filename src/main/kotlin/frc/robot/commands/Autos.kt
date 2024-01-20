package frc.robot.commands

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.lib.basics.SwerveDriveBase
import frc.lib.utils.TrajectoryOrchestrator
import frc.robot.subsystems.ExampleSubsystem

class Autos private constructor() {
    init {
        throw UnsupportedOperationException("This is a utility class!")
    }

    companion object {
        /** Example static factory for an autonomous command.  */
        fun exampleAuto(subsystem: ExampleSubsystem): Command {
            return Commands.sequence(subsystem.exampleMethodCommand(), ExampleCommand(subsystem))
        }

        fun moveOutTheWayAuto(drive: SwerveDriveBase): Command{
            val color = DriverStation.getAlliance()
            var to = Pose2d(2.1336, 0.0, Rotation2d.fromDegrees(0.0)).relativeTo(drive.getPose())
            color?.ifPresent{
                color ->
               if(color == DriverStation.Alliance.Red){
                    to = Pose2d(-2.1336, 0.0, Rotation2d.fromDegrees(0.0)).relativeTo(drive.getPose())
                }

            }
            return TrajectoryOrchestrator.beelineCommand(
                drive,
                to
            )

        }

    }
}
