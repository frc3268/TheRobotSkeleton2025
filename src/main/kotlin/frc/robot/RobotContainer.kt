package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.lib.basics.SwerveDriveBase
import frc.robot.commands.Autos
import frc.robot.commands.SwerveJoystickDrive
import frc.robot.subsystems.ClimberSubsystem
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.ShooterSubsystem

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {

    private val ShuffleboardTab = Shuffleboard.getTab("General")
    private val TroubleShootingTab = Shuffleboard.getTab("TroubleShooting")




    // The robot's subsystems and commands are defined here...
    val driveSubsystem = SwerveDriveBase(Pose2d())
    val intakeSubsystem = IntakeSubsystem()
    val shooterSubsystem = ShooterSubsystem()
    val climberSubsystem = ClimberSubsystem()


    // Replace with CommandPS4Controller or CommandJoystick if needed
    private val driverController = CommandXboxController(Constants.OperatorConstants.kDriverControllerPort)

    val autochooser = SendableChooser<Command>()
    val startingPositionChooser = SendableChooser<Pose2d?>()


    //this is the command called when teleop mode is enabled
     val teleopCommand = SwerveJoystickDrive(
        driveSubsystem,
        { driverController.getRawAxis(1) },
        { driverController.getRawAxis(0) },
        { -driverController.getRawAxis(2) },
        { !driverController.leftBumper().asBoolean }
    )
    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        driveSubsystem.defaultCommand = teleopCommand

        ShuffleboardTab
            .add("Autonomous Mode", autochooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1)

        autochooser.setDefaultOption("taxi", Autos.taxiAuto(driveSubsystem))
        autochooser.addOption("shoot to speaker", Autos.driveUpAndShootSpeakerCommand(driveSubsystem, intakeSubsystem, shooterSubsystem))

        ShuffleboardTab
            .add("Starting Position", startingPositionChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 1)
            .withSize(2, 1)

        //todo! make these into the real poses from the field. How? idk
        startingPositionChooser.setDefaultOption("None", null)
        startingPositionChooser.setDefaultOption("Red 1", null)
        startingPositionChooser.setDefaultOption("Red 2", null)
        startingPositionChooser.setDefaultOption("Red 3", null)
        startingPositionChooser.setDefaultOption("Blue 1", null)
        startingPositionChooser.setDefaultOption("Blue 2", null)
        startingPositionChooser.setDefaultOption("Blue 3", null)

        ShuffleboardTab.add("Drive and Shoot: Speaker", Autos.driveUpAndShootSpeakerCommand(driveSubsystem, intakeSubsystem, shooterSubsystem)).withWidget(BuiltInWidgets.kCommand)
        ShuffleboardTab.add("Get Floor Note", Autos.intakeAndUpCommand(intakeSubsystem)).withWidget(BuiltInWidgets.kCommand)
        ShuffleboardTab.add("Get Source Note: Closer To Baseline", Autos.goToSourceAndIntakeCommand(driveSubsystem, true, shooterSubsystem)).withWidget(BuiltInWidgets.kCommand)
        ShuffleboardTab.add("Get Source Note: Not Closer To Baseline", Autos.goToSourceAndIntakeCommand(driveSubsystem, false, shooterSubsystem)).withWidget(BuiltInWidgets.kCommand)

        /*
      TODO: add 3 buttons (pos 1, 2, 3), to reset the robot's pose in the event of a camera failure
      URGENT URGENT!
       */


        TroubleShootingTab.add("Zero arm encoder", intakeSubsystem.zeroArmEncoderCommand()).withWidget(BuiltInWidgets.kCommand)



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

        // Schedule exampleMethodCommand when the Xbox controller's B button is pressed,
        // cancelling on release.
        //driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand())
        driverController.a().onTrue(Autos.shootSpeakerCommand(intakeSubsystem, shooterSubsystem))
        driverController.b().onTrue(Autos.intakeAndUpCommand(intakeSubsystem))
        driverController.x().onTrue(Autos.sourceIntakeCommand(shooterSubsystem))
        driverController.y().onTrue(Autos.shootAmpCommand(intakeSubsystem, shooterSubsystem))
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
