package frc.lib.core

import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.SwerveJoystickDrive
import frc.lib.swerve.SwerveDriveBase
import frc.robot.Constants

/** A Basic core singleton. *WARNING: You must call [initCore] before using or acessing this singleton!* */
object robotCore {
    /** The drive subsystem. Must be accessed after calling [initCore] */
    lateinit var driveSubsystem: SwerveDriveBase
    lateinit var driverController: CommandXboxController
    val autoChooser = SendableChooser<Command>()

    val GeneralTab = Shuffleboard.getTab("General")
    val CalibrationTab = Shuffleboard.getTab(Constants.CALIBRATION_TAB)

    
    fun goto(goal: FieldLocation): Command {
        return SwerveAutoDrive(
            {goal},
            driveSubsystem
        )
    }

    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    val autonomousCommand: Command
        get() {
            return autochooser.selected
        }

    
    fun initCore() {
        // initialize HAL
        check(HAL.initialize(500, 0)) { "Failed to initialize. Terminating." }

        // report robot language as Kotlin
        // 6 Means kotlin in French
        HAL.report(FRCNetComm.tResourceType.kResourceType_Language, 6)

        driveSubsystem = SwerveDriveBase(Pose2d());
        driverController = CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT)

        driverController.povDown().onTrue(Routines.inchBack(driveSubsystem))
        driverController.povUp().onTrue(Routines.inchForward(driveSubsystem))
        driverController.povRight().onTrue(Routines.inchRight(driveSubsystem))
        driverController.povLeft().onTrue(Routines.inchLeft(driveSubsystem))


        driveSubsystem.defaultCommand = teleopCommand
    
        autoChooser.addOption("Do nothing", WaitCommand(3.0))
        autoChooser.setDefaultOption("Taxi", SwerveJoystickDrive(driveSubsystem, {1.0}, {0.0}, {0.0}, {false} ).withTimeout(1.0))
    }

    // Idk if this should be called in initCore or not
    fun initWidgets() {
        GeneralTab
            .add("Autonomous Mode", autochooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1)
    }
}
