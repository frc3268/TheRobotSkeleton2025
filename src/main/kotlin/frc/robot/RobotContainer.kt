package frc.robot

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.shuffleboard.*
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
import frc.lib.*
import frc.lib.FieldPositions.obstacles
import frc.lib.swerve.SwerveDriveBase
import frc.lib.swerve.SwerveDriveConstants
import frc.robot.algaeintake.AlgaeIntakeIOKraken
import frc.robot.algaeintake.AlgaeIntakeSubsystem
import frc.robot.commands.*
import frc.robot.coralintake.CoralIntakeIO
import frc.robot.coralintake.CoralIntakeIOKraken
import frc.robot.coralintake.CoralIntakeSubsystem
import frc.robot.elevator.ElevatorIOKraken
import frc.robot.elevator.ElevatorSubsystem
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.decodeFromStream
import java.io.File
import java.util.concurrent.atomic.AtomicReference
import java.util.function.Supplier
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    private val GeneralTab = Shuffleboard.getTab("General")
    private val thingy = GeneralTab.add("Thing", 0.0).entry
    private val TroubleshootingTab = Shuffleboard.getTab(Constants.TROUBLESHOOTING_TAB)

    val driveSubsystem = SwerveDriveBase(Pose2d())

    private val driverController = CommandPS4Controller(Constants.OperatorConstants.kDriverControllerPort)

    // There must be a better way to do this
    var coralIntakeSubsystem: CoralIntakeSubsystem? = null
    var elevatorSubsystem: ElevatorSubsystem? = null
    var algaeIntakeSubsystem: AlgaeIntakeSubsystem? = null


    val autochooser = SendableChooser<Command>()
    val leftBumperChooser = SendableChooser<Command>()
    val xButtonChooser = SendableChooser<Command>()


    val teleopCommand = SwerveJoystickDrive(
        driveSubsystem,
        { driverController.getRawAxis(1) },
        { -driverController.getRawAxis(0) },
        { -driverController.getRawAxis(2) },
        { true }
    )


    //type is () -> Command because otherwise CommandScheduler complains that each one has already been scheduled
    val autos: MutableMap<String, () -> Command> = mutableMapOf(
        "goToProcessor" to { goto(FieldPositions.processor) }
    )


    val grid = Json.decodeFromStream<gridFile>(
        File(Filesystem.getDeployDirectory().toString() + "/pathplanner/navgrid.json").inputStream()).grid

    fun goto(goal: FieldLocation): Command {
        val color = DriverStation.getAlliance()
        val to =
            if (color.isPresent && color.get() == DriverStation.Alliance.Red)
                goal.red
            else
                goal.blue
        return SwerveAutoDrive(
            {to},
            driveSubsystem,
            grid
        )
    }



    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {

        if (Constants.mode == Constants.States.REAL) {
            coralIntakeSubsystem = CoralIntakeSubsystem(CoralIntakeIOKraken())
            algaeIntakeSubsystem = AlgaeIntakeSubsystem(AlgaeIntakeIOKraken())
            elevatorSubsystem = ElevatorSubsystem(ElevatorIOKraken())
        }
        else {
            println("Simulated subsystems do not exist as no IOClass for them exists!")
        }
        driveSubsystem.defaultCommand = teleopCommand

        GeneralTab
            .add("Autonomous Mode", autochooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1)
        GeneralTab
            .add("LB Command", leftBumperChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(2, 0)
            .withSize(2, 1)

        for (file:File in File(Filesystem.getDeployDirectory().toString() + "/paths").listFiles()?.filter { it.isFile }!!){
            if(autochooser.selected == null){
                autochooser.setDefaultOption(file.name,Json.decodeFromStream<AutoSequence>(
                    file.inputStream()
                ).toCommandGroup(autos))

                Shuffleboard.getTab("General").addString("something35", {"test"})
            }
            else {
                autochooser.addOption(
                    file.name, Json.decodeFromStream<AutoSequence>(
                        file.inputStream()
                    ).toCommandGroup(autos)
                )
            }
        }

        for (file:File in File(Filesystem.getDeployDirectory().toString() + "/buttons").listFiles()?.filter { it.isFile }!!){
            GeneralTab.add(file.name,Json.decodeFromStream<AutoSequence>(
                file.inputStream()
            ).toCommandGroup(autos)).withWidget(BuiltInWidgets.kCommand)
        }
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
}
