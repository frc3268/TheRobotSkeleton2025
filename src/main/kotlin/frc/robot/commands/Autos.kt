package frc.robot.commands

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.lib.basics.SwerveDriveBase
import frc.lib.utils.TrajectoryOrchestrator
import frc.lib.utils.rotation2dFromDeg
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

        fun taxiAuto(drive: SwerveDriveBase): Command{
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

        fun goToSpeaker(drive:SwerveDriveBase):Command{
            val color = DriverStation.getAlliance()
            //todo: fix x
            var to = Pose2d(1.0, 5.547868, 0.0.rotation2dFromDeg())
            color?.ifPresent { color ->
                if (color == DriverStation.Alliance.Red) {
                    to = Pose2d(15.579342, 5.547868, 180.0.rotation2dFromDeg())
                }
            }
            return TrajectoryOrchestrator.beelineCommand(
                drive,
                to
            )

        }

    }
}
