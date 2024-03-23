package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.shuffleboard.*
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.lib.basics.SwerveDriveBase
import frc.robot.commands.*
import frc.robot.subsystems.*

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
    val lbChooser = SendableChooser<Command>()

    val teleopCommand = SwerveJoystickDrive(
        driveSubsystem,
        { driverController.getRawAxis(1) },
        { -driverController.getRawAxis(0) },
        { -driverController.getRawAxis(4) },
        { true }
    )

    val ring1BooleanBox = GeneralTab.add("Collect ring 1?", false).withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(1, 1)
    val ring2BooleanBox = GeneralTab.add("Collect ring 2?", false).withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(2, 1)
    val ring3BooleanBox = GeneralTab.add("Collect ring 3?", false).withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(3, 1)
    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        driveSubsystem.defaultCommand = teleopCommand

        GeneralTab
            .add("Autonomous Mode", autochooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1)
        GeneralTab
            .add("LB Command", lbChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(2, 0)
            .withSize(2, 1)


        autochooser.setDefaultOption("Taxi", Autos.taxiAuto(driveSubsystem))
        autochooser.addOption("Do nothing", WaitCommand(1.0))
        autochooser.addOption("Shoot to speaker", Autos.shootSpeakerCommand(intakeSubsystem, shooterSubsystem))
        autochooser.addOption("Shoot to speaker + taxi", Autos.shootSpeakerCommand(intakeSubsystem, shooterSubsystem).andThen(Autos.taxiAuto(driveSubsystem)))
        autochooser.addOption("Shoot, Intake, Shoot", Autos.driveUpShootSpeakerAndReturnToRingsCommand(driveSubsystem, intakeSubsystem, shooterSubsystem))
        autochooser.addOption("Shoot, Take rings and Shoot", Autos.collectStartingRingsAndShoot(driveSubsystem, intakeSubsystem, shooterSubsystem, 1, arrayOf(ring1BooleanBox.entry, ring2BooleanBox.entry, ring3BooleanBox.entry)))

        //test individual commands
        autochooser.addOption("test go to speaker (bottom)", Autos.goToSpeakerCommand(driveSubsystem, 1))
        autochooser.addOption("test go to speaker (middle)", Autos.goToSpeakerCommand(driveSubsystem, 2))
        autochooser.addOption("test go to speaker (top)", Autos.goToSpeakerCommand(driveSubsystem, 3))
        autochooser.addOption("test go to amp", Autos.goToAmpCommand(driveSubsystem))
        autochooser.addOption("test go to source closerToBaseLine=false", Autos.goToSourceCommand(driveSubsystem, false))
        autochooser.addOption("test go to source closerToBaseLine=false", Autos.goToSourceCommand(driveSubsystem, true))
        autochooser.addOption("test go within speaker (obsolete?)", Autos.goWithinSpeakerCommand(driveSubsystem))
        autochooser.addOption("test intake and up", Autos.intakeAndUpCommand(intakeSubsystem))
        autochooser.addOption("test intake from ground", Autos.intakeNoteCommand(intakeSubsystem))
        autochooser.addOption("climber up", Autos.climberUp(leftClimberSubsystem, rightClimberSubsystem))
        autochooser.addOption("climber down", Autos.climberDown(leftClimberSubsystem, rightClimberSubsystem))
        autochooser.addOption("climber stop", Autos.climberStop(leftClimberSubsystem, rightClimberSubsystem))
        autochooser.addOption("test shootSpeakerCommand", Autos.shootSpeakerCommand(intakeSubsystem, shooterSubsystem))
        autochooser.addOption("test Shooter only", Autos.testShooterCommand(shooterSubsystem))
        autochooser.addOption("test shoot amp", Autos.shootAmpCommand(intakeSubsystem, shooterSubsystem))
        autochooser.addOption("test source intake", Autos.sourceIntakeCommand(shooterSubsystem, intakeSubsystem))

        GeneralTab
        .addCamera("Driver Camera", "USB Camera 0")
        .withPosition(4, 0)
                .withSize(3, 3);

        //lb chooser
        lbChooser.setDefaultOption("Do nothing", WaitCommand(0.0))
        lbChooser.addOption(" go to speaker (bottom)", Autos.goToSpeakerCommand(driveSubsystem, 1))
        lbChooser.addOption(" go to speaker (middle)", Autos.goToSpeakerCommand(driveSubsystem, 2))
        lbChooser.addOption(" go to speaker (top)", Autos.goToSpeakerCommand(driveSubsystem, 3))

        GeneralTab.add("Source Intake", Autos.sourceIntakeCommand(shooterSubsystem, intakeSubsystem))
                .withPosition(3, 2)

        GeneralTab.add("LR down", Autos.climberDown(leftClimberSubsystem, rightClimberSubsystem)).withWidget(BuiltInWidgets.kCommand)
                .withPosition(2, 2)
        GeneralTab.add("LR up", Autos.climberUp(leftClimberSubsystem, rightClimberSubsystem)).withWidget(BuiltInWidgets.kCommand)
                .withPosition(1, 2)
        GeneralTab.add("LR stop", Autos.climberStop(leftClimberSubsystem, rightClimberSubsystem)).withWidget(BuiltInWidgets.kCommand)
                .withPosition(0, 2)
        // Troubleshooting tab holds manual controls for the climber and a reset for the arm encoder
        TroubleshootingTab.add("L down test", leftClimberSubsystem.testdown()).withWidget(BuiltInWidgets.kCommand)
        TroubleshootingTab.add("L up test", leftClimberSubsystem.testup()).withWidget(BuiltInWidgets.kCommand)

        TroubleshootingTab.add("R down test", rightClimberSubsystem.testdown()).withWidget(BuiltInWidgets.kCommand)
        TroubleshootingTab.add("R up test", rightClimberSubsystem.testup()).withWidget(BuiltInWidgets.kCommand)

        TroubleshootingTab.add("arm enc zero", intakeSubsystem.zeroArmEncoderCommand()).withWidget(BuiltInWidgets.kCommand)

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
            runs intake and up sequence
            1. arm goes down
            2. intake turns on until
            3. a piece has been detected in the intake, then
            4. intake stops and arm goes up
         */
        driverController.leftTrigger().onTrue(Autos.intakeAndUpCommand(intakeSubsystem))
        
        /*
        LB (Go to speaker): run a given go to command
        you can pick a place to go to in the lbchooser
        (reasoning: driver is able to focus on other things while robot goes to speaker autonomously and drifting is not an issue)
         */
        driverController.leftBumper().onTrue(lbChooser.selected)

        /*
        RT (Shoot Toggle):
            1) Rev up shooter
         */
        driverController.rightTrigger().onTrue(intakeSubsystem.runIntakeAtSpeed(IntakeSubsystem.OUTTAKE_SPEED))
        driverController.rightTrigger().onFalse(intakeSubsystem.stopIntake())

        /*
        RB - REV UP SHOOTER (TOGGLE)
         */
        driverController.rightBumper().toggleOnTrue(Commands.startEnd({shooterSubsystem.leftFlywheelMotor.set(-1.0)
            shooterSubsystem.rightFlywheelMotor.set(-1.0)}, {shooterSubsystem.leftFlywheelMotor.stopMotor()
            shooterSubsystem.rightFlywheelMotor.stopMotor()}, shooterSubsystem))

        /*
        Y (EMERGENCY STOP): Stop the intake gears, the arm, and the shooter.
        (The intention is to be able to prevent damage if the encoder is faulty and damaging any moving parts.)
         */
        driverController.y().onTrue(Autos.emergencyStopCommand(shooterSubsystem, intakeSubsystem, leftClimberSubsystem, rightClimberSubsystem))

        /*
        A runs intake when held
         */
        driverController.a().onTrue(intakeSubsystem.runIntakeAtSpeed(IntakeSubsystem.INTAKE_SPEED))
        driverController.a().onFalse(intakeSubsystem.stopIntake())

        /*
        POV up and down bring arm up and down
         */
        driverController.povUp().onTrue(intakeSubsystem.armUpCommand())
        driverController.povDown().onTrue(intakeSubsystem.armDownCommand())
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
