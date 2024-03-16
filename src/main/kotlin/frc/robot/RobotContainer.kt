package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.shuffleboard.*
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.lib.FieldLocation
import frc.lib.FieldPositions
import frc.lib.SwerveDriveBase
import frc.lib.rotation2dFromDeg
import frc.robot.commands.*
import frc.robot.subsystems.*
import java.util.function.Supplier
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.sin

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    private val GeneralTab = Shuffleboard.getTab("General")
    private val TroubleshootingTab = Shuffleboard.getTab("Troubleshooting")

    val driveSubsystem = SwerveDriveBase(Pose2d())
    val intakeSubsystem = IntakeSubsystem()
    val shooterSubsystem = ShooterSubsystem()
    val leftClimberSubsystem = LeftClimberSubsystem()
    val rightClimberSubsystem = RightClimberSubsystem()

    private val driverController = CommandXboxController(Constants.OperatorConstants.kDriverControllerPort)

    val autochooser = SendableChooser<Command>()

    val teleopCommand = SwerveJoystickDrive(
        driveSubsystem,
        { driverController.getRawAxis(1) },
        { -driverController.getRawAxis(0) },
        { -driverController.getRawAxis(4) },
        { true }
    )

    fun goto(goal: FieldLocation): Command {
        val color = DriverStation.getAlliance()
        val to =
            if (color.isPresent && color.get() == DriverStation.Alliance.Red)
                goal.red
            else
                goal.blue
        return SequentialCommandGroup(
            driveSubsystem.moveToPoseCommand(to),
            InstantCommand({ driveSubsystem.stop() })
        )
    }

    fun goToSpeakerCloser(): Command {
        var startingPose = Pose2d()
        return runOnce({
            startingPose = driveSubsystem.getPose()
        }, driveSubsystem).andThen(
        //red
        goto(
            FieldPositions.closest(
                startingPose,
                listOf(
                    FieldPositions.speakerRight,
                    FieldPositions.speakerCenter,
                    FieldPositions.speakerLeft
                )
            )
        )
        )
    }


    val intakeAndUpCommand: Command =
        SequentialCommandGroup(
            intakeSubsystem.armDownCommand(),
            intakeSubsystem.takeInCommand(),
            intakeSubsystem.stopIntake(),
            intakeSubsystem.armUpCommand(),
        )

    val intakeNoteCommand: Command =
        SequentialCommandGroup(
            intakeSubsystem.takeInCommand(),
            intakeSubsystem.stopIntake()
        )

    val climberUp: ParallelCommandGroup =
        ParallelCommandGroup(
            leftClimberSubsystem.up(),
            rightClimberSubsystem.up()
        )

    val climberDown: ParallelCommandGroup =
        ParallelCommandGroup(
            leftClimberSubsystem.down(),
            rightClimberSubsystem.down()
        )

    val climberStop: ParallelCommandGroup =
        ParallelCommandGroup(
            leftClimberSubsystem.stop(),
            rightClimberSubsystem.stop()
        )

    val shootSpeakerCommand: Command =
        SequentialCommandGroup(
            shooterSubsystem.shootCommand(),
            WaitCommand(1.0),
            intakeSubsystem.takeOutCommand(),
            WaitCommand(1.2),
            shooterSubsystem.stopCommand(),
            intakeSubsystem.stopIntake()
        )

    val shootAmpCommand: Command =
        SequentialCommandGroup(
            shooterSubsystem.ampCommand(),
            intakeSubsystem.takeOutCommand(),
            shooterSubsystem.stopCommand()
        )

    val sourceIntakeCommand: Command =
        SequentialCommandGroup(
            intakeSubsystem.armUpCommand(),
            shooterSubsystem.takeInCommand(),
            intakeSubsystem.runIntakeCommand(),
            WaitCommand(0.5),
            intakeSubsystem.stopIntake()
        )

    val emergencyStopCommand: Command =
        SequentialCommandGroup(
            shooterSubsystem.stopCommand(),
            intakeSubsystem.stopAllCommand()
        )

    val autos = mapOf(
        "gotoSpeaker" to goToSpeakerCloser(),
        "gotoSpeakerCenter" to goto(FieldPositions.speakerCenter),
        "gotoSpeakerRight" to goto(FieldPositions.speakerRight),
        "gotoSpeakerLeft" to goto(FieldPositions.speakerLeft),
        "gotoAmp" to goto(FieldPositions.amp),
        "goToSourceCloserToBaseline" to goto(FieldPositions.sourceBaseline),
        "goToSourceFurtherFromBaseline" to goto(FieldPositions.sourceNotBaseline),
        "goToRing" to WaitCommand(1.0),
        //todo: other rings
        "shootSpeaker" to shootSpeakerCommand,
        "shootAmp" to shootAmpCommand,
        "intakeAndUp" to intakeAndUpCommand,
        "climbersUp" to climberUp,
        "climbersDown" to climberDown
    )

    fun loadSequence(filepath:String) : SequentialCommandGroup{
        return SequentialCommandGroup()
    }



    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        driveSubsystem.defaultCommand = teleopCommand

        GeneralTab
            .add("Autonomous Mode", autochooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1)

        // Troubleshooting tab holds manual controls for the climber and a reset for the arm encoder
        TroubleshootingTab.add("CLIMBER L down", leftClimberSubsystem.testdown()).withWidget(BuiltInWidgets.kCommand)
        TroubleshootingTab.add("CLIMBER L up", leftClimberSubsystem.testup()).withWidget(BuiltInWidgets.kCommand)

        TroubleshootingTab.add("CLIMBER R down", rightClimberSubsystem.testdown()).withWidget(BuiltInWidgets.kCommand)
        TroubleshootingTab.add("CLIMBER R up", rightClimberSubsystem.testup()).withWidget(BuiltInWidgets.kCommand)

        TroubleshootingTab.add("Zero ARM ENCODER", intakeSubsystem.zeroArmEncoderCommand()).withWidget(BuiltInWidgets.kCommand)

        TroubleshootingTab.add("CLIMBERS reset",climberStop).withWidget(BuiltInWidgets.kCommand)
        TroubleshootingTab.add("CLIMBERS stop", leftClimberSubsystem.stop().alongWith(rightClimberSubsystem.stop())).withWidget(BuiltInWidgets.kCommand)


        // Configure the trigger bindings
        configureBindings()
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * [Trigger#Trigger(java.util.function.BooleanSupplier)] constructor with an arbitrary
     * predicate, or via the named factories in [edu.wpi.first.wpilibj2.command.button.CommandGenericHID]'s subclasses for
     * [CommandXboxController]/[edu.wpi.first.wpilibj2.command.button.CommandPS4Controller] controllers
     * or [edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings() {
        // Schedule ExampleCommand when exampleCondition changes to true
        //Trigger { exampleSubsystem.exampleCondition() }.onTrue(ExampleCommand(exampleSubsystem))

        /*
        LT (Intake):
            runs intake
            (not arm!)
         */
        driverController.leftTrigger().onTrue(intakeSubsystem.runIntakeCommand())
        driverController.leftTrigger().onFalse(intakeSubsystem.stopIntake())

        /*
        RT (Shoot):
            1) Rev up shooter
            2) Run intake in reverse to feed it into shooter
            This assumes the arm is already up. If it's down, the note will be shot back onto the ground.
         */
        driverController.rightTrigger().onTrue(shooterSubsystem.shootCommand())

        /*
        LB: Arm up
         */
        driverController.leftBumper().onTrue(intakeSubsystem.armDownCommand())

        /*
        RB: Arm Up
         */
        driverController.rightBumper().onTrue(intakeSubsystem.armUpCommand())

        /*
        Y (EMERGENCY STOP): Stop the intake gears, the arm, and the shooter.
        (The intention is to be able to prevent damage if the encoder is faulty and damaging any moving parts.)
         */
        driverController.y().onTrue(emergencyStopCommand)

        /*
        A runs outake
         */
        driverController.a().onTrue(intakeSubsystem.runOnceOuttake())
        driverController.a().onFalse(intakeSubsystem.stopIntake())

        /*
        X does source intake
        arm up
        run shooter in reverse
        intake
        stop intake
        stop shooter
         */
        driverController.x().onTrue(sourceIntakeCommand)

       /*
       B does arm down, intake note, arm up
        */
        driverController.b().onTrue(intakeAndUpCommand)

        /*
        POV up and down bring arm up and down
         */
        //driverController.povUp().onTrue(intakeSubsystem.armUpCommand())
        //driverController.povDown().onTrue(intakeSubsystem.armDownCommand())
    }

    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    val autonomousCommand: Command
        get() {
            // wait 3 seconds...
            return autochooser.selected
        }
}
