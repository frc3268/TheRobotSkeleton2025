package frc.robot.commands

import edu.wpi.first.math.geometry.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.*
import frc.lib.basics.SwerveDriveBase
import frc.lib.utils.*
import frc.robot.subsystems.*
import kotlin.math.*

class Autos private constructor() {
    init {
        throw UnsupportedOperationException("This is a utility class!")
    }

    companion object {
        /** Example static factory for an autonomous command.  */
        fun exampleAuto(subsystem: ExampleSubsystem): Command {
            return Commands.sequence(subsystem.exampleMethodCommand(), ExampleCommand(subsystem))
        }

        /**
         * Drives to [goalIfRed] if the robot is on the red team, otherwise [goalOtherwise].
         *
         * [goalOtherwise] is the default if there is no color.
         */
        fun goto(drive: SwerveDriveBase, goalIfRed: Pose2d, goalOtherwise: Pose2d): Command {
            val color = DriverStation.getAlliance()
            val to =
                if (color.isPresent && color.get() == DriverStation.Alliance.Red)
                    goalIfRed
                else
                    goalOtherwise
            return TrajectoryOrchestrator.beelineCommand(drive, to)
        }

        fun taxiAuto(drive: SwerveDriveBase): Command =
            goto(
                drive,
                Pose2d(drive.getPose().x-2.1336, drive.getPose().y, Rotation2d.fromDegrees(0.0)),
                Pose2d(drive.getPose().x + 2.1336, drive.getPose().y, Rotation2d.fromDegrees(180.0))
            )

        fun goToSpeakerCommand(drive: SwerveDriveBase): Command =
            goto(
                drive,
                Pose2d(15.256, 5.547868, 0.0.rotation2dFromDeg()),
                Pose2d(1.6096, 5.547868, 180.0.rotation2dFromDeg())
            )

        fun goToAmpCommand(drive: SwerveDriveBase): Command =
            goto(
                drive,
                Pose2d(14.929358, 8.2042, 270.0.rotation2dFromDeg()),
                Pose2d(1.84404, 8.2042, 270.0.rotation2dFromDeg())
            )

        fun goToSourceCommand(drive: SwerveDriveBase, closerToBaseLine: Boolean): Command =
            goto(
                drive,

                if (closerToBaseLine) Pose2d(0.356108, 0.883666, 60.0.rotation2dFromDeg())
                else Pose2d(1.461516, 0.245872, 60.0.rotation2dFromDeg()),

                if (closerToBaseLine) Pose2d(15.079472, 0.245872, 120.0.rotation2dFromDeg())
                else Pose2d(16.185134, 0.883666, 120.0.rotation2dFromDeg())
            )

        fun goToSourceAndIntakeCommand(drive: SwerveDriveBase, closerToBaseLine: Boolean, shooter: ShooterSubsystem, intake: IntakeSubsystem): Command =
            SequentialCommandGroup(
                goToSourceCommand(drive, closerToBaseLine),
                sourceIntakeCommand(shooter, intake)
            )

        fun goWithinSpeakerCommand(drive: SwerveDriveBase): Command {
            val color = DriverStation.getAlliance()
            //todo: fix x
            val to =
                if (color.isPresent && color.get() == DriverStation.Alliance.Red)
                    Pose2d(14.579342, 5.547868, 180.0.rotation2dFromDeg())
                else
                    Pose2d(0.0, 5.547868, 0.0.rotation2dFromDeg())
            //todo: make this real
            val pose = drive.getPose()
            val c = 1.0
            val theta = atan((pose.y - to.y) / (pose.x - to.x)).rotation2dFromDeg()
            return TrajectoryOrchestrator.beelineCommand(
                    drive, Pose2d(to.x + cos(theta.radians) * c, to.y + sin(theta.radians) * c, theta + pose.rotation))
        }

        fun driveUpAndShootSpeakerCommand(drive: SwerveDriveBase, intake: IntakeSubsystem, shooter: ShooterSubsystem): Command =
            SequentialCommandGroup(
                goToSpeakerCommand(drive),
                shootSpeakerCommand(intake, shooter)
            )

        fun intakeAndUpCommand(intake: IntakeSubsystem): Command =
            SequentialCommandGroup(
                    intake.armDownCommand(),
                    intake.takeInCommand(),
                    intake.stopIntake(),
                    intake.armUpCommand(),
            )

        fun climberUp(left: LeftClimberSubsystem, right: RightClimberSubsystem): ParallelCommandGroup =
            ParallelCommandGroup(
                left.up(),
                right.up()
            )

        fun climberDown(left: LeftClimberSubsystem, right: RightClimberSubsystem): ParallelCommandGroup =
            ParallelCommandGroup(
                left.down(),
                right.down()
            )

        fun climberStop(left: LeftClimberSubsystem, right: RightClimberSubsystem): ParallelCommandGroup =
            ParallelCommandGroup(
                left.stop(),
                right.stop()
            )

        fun shootSpeakerCommand(intake: IntakeSubsystem, shooter: ShooterSubsystem): Command =
            SequentialCommandGroup(
                    shooter.shootCommand(),
                    WaitCommand(1.0),
                    intake.takeOutCommand(),
                    WaitCommand(1.2),
                    shooter.stopCommand(),
                    intake.stopIntake()
            )

        fun shootAmpCommand(intake: IntakeSubsystem, shooter: ShooterSubsystem): Command =
            SequentialCommandGroup(
                shooter.ampCommand(),
                intake.takeOutCommand(),
                shooter.stopCommand()
            )

        fun sourceIntakeCommand(shooter: ShooterSubsystem, intake: IntakeSubsystem): Command {
            return SequentialCommandGroup(
                    intake.armUpCommand(),
                    shooter.takeInCommand(),
                    intake.takeInCommand()
            )
        }

        fun driveUpAndIntakeSourceCommand(drive: SwerveDriveBase, shooter: ShooterSubsystem, closerToBaseLine: Boolean, intake: IntakeSubsystem): Command =
            SequentialCommandGroup(
                goToSourceCommand(drive, closerToBaseLine),//fix closer to baseline
                sourceIntakeCommand(shooter, intake)
            )

        fun emergencyStopCommand(shooter: ShooterSubsystem, intake: IntakeSubsystem): Command =
            SequentialCommandGroup(
                shooter.stopCommand(),
                intake.stopAllCommand()
            )
    }
}
