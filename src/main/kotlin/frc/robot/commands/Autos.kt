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
import kotlin.math.atan
import kotlin.math.cos

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

        fun goToAmp(drive:SwerveDriveBase):Command{
            val radius:Int = 0 //to do: calibrate distance from speaker
            val color = DriverStation.getAlliance()
            //todo: fix x
            var to = Pose2d(1.84404, 8.2042, 270.0.rotation2dFromDeg())
            color?.ifPresent { color ->
                if (color == DriverStation.Alliance.Red) {
                    to = Pose2d(14.929358, 8.2042, 270.0.rotation2dFromDeg())
                }
            }
            return TrajectoryOrchestrator.beelineCommand(
                drive,
                to
            )

        }

        fun goToSource(drive:SwerveDriveBase, closerToBaseLine: Boolean):Command{
            val color = DriverStation.getAlliance()
            var to = Pose2d(0.356108, 0.883666, 60.0.rotation2dFromDeg())
            if(!closerToBaseLine){
                to = Pose2d(1.461516, 0.245872, 60.0.rotation2dFromDeg())
            }
            color?.ifPresent{ color ->
                if (color == DriverStation.Alliance.Blue){
                    to = Pose2d(15.079472, 0.245872, 120.0.rotation2dFromDeg())
                    if(!closerToBaseLine){
                        to = Pose2d(16.185134, 0.883666, 120.0.rotation2dFromDeg())
                    }
                }

            }
            return TrajectoryOrchestrator.beelineCommand(
                drive, to
            )
        }

        fun goWithinSpeaker(drive:SwerveDriveBase): Command {
            val color = DriverStation.getAlliance()
            //todo: fix x
            var to = Pose2d(0.0, 5.547868, 0.0.rotation2dFromDeg())
            color?.ifPresent { color ->
                if(color == DriverStation.Alliance.Red){
                    to = Pose2d(14.579342, 5.547868, 180.0.rotation2dFromDeg())
                }
            }
            //todo: make this real
            val pose = drive.getPose()
            val c = 1.0
            val x = pose.x - to.x
            val y = pose.y - to.y
            val theta = atan(x/y).rotation2dFromDeg()
            val nx = theta.cos*c
            val ny = theta.sin*c
            return TrajectoryOrchestrator.beelineCommand(
                drive, Pose2d(pose.x + nx, pose.y+ny, theta)
            )
        }

        //go to speaker
        //shootcommand()

    }
}
