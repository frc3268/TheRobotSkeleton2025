package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.lib.basics.SwerveDriveBase
import frc.lib.utils.TrajectoryOrchestrator
import frc.robot.commands.Autos
import frc.robot.commands.SwerveJoystickDrive

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {

    private val ShuffleboardTab = Shuffleboard.getTab("General")




    // The robot's subsystems and commands are defined here...
    //todo: change this to reflect a field position. Maybe use a constant?
    val driveSubsystem:SwerveDriveBase = SwerveDriveBase(Pose2d())


    // Replace with CommandPS4Controller or CommandJoystick if needed
    private val driverController = Joystick(Constants.OperatorConstants.kDriverControllerPort)

    val autochooser:SendableChooser<Command> = SendableChooser<Command>()

    val autoCommand:Command = TrajectoryOrchestrator.buildSwerveTrajectory(
        Pose2d(0.0,0.0, Rotation2d(0.0)),
        Pose2d(0.0, 1.0, Rotation2d.fromDegrees(0.0)),
        mutableListOf(
            Translation2d(1.0, 0.0),
            Translation2d(1.0, 1.0)
        ),
        driveSubsystem
    )

    //this is the command called when teleop mode is enabled
     val teleopCommand = SwerveJoystickDrive(
        driveSubsystem,
        { driverController.getRawAxis(1) },
        { driverController.getRawAxis(0) },
        { -driverController.getRawAxis(2) },
        { !driverController.triggerPressed }
    )
    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        //todo: make real and test
        ShuffleboardTab
            .add("Autonomous Mode", autochooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1);
        driveSubsystem.setDefaultCommand(teleopCommand)
        autochooser.setDefaultOption("taxi", Autos.taxiAuto(driveSubsystem))
        autochooser.addOption("test", WaitCommand(3.0))
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
    }

    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    val autonomousCommand: Command
        get() {
            // wait 3 seconds...
            return autoCommand
        }
}
