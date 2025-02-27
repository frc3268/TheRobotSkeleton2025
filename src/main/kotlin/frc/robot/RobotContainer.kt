package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
import frc.lib.AutoSequence
import frc.lib.FieldLocation
import frc.lib.FieldPositions
import frc.lib.swerve.SwerveDriveBase
import frc.robot.algaeintake.AlgaeIntakeSubsystem
import frc.robot.climber.ClimberSubsystem
import frc.robot.commands.*
import frc.robot.coralintake.CoralIntakeSubsystem
import frc.robot.elevator.ElevatorSubsystem
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.decodeFromStream
import java.io.File


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

    // There must be a better way to do this! Oh well.
    var coralIntakeSubsystem: CoralIntakeSubsystem? = null
    var elevatorSubsystem: ElevatorSubsystem? = null
    var algaeIntakeSubsystem: AlgaeIntakeSubsystem? = null
    var climberSubsystem: ClimberSubsystem? = null


    val autochooser = SendableChooser<Command>()
    val leftBumperChooser = SendableChooser<Command>()
    val xButtonChooser = SendableChooser<Command>()


    val teleopCommand = SwerveJoystickDrive(
        driveSubsystem,
        { driverController.getRawAxis(1) },
        { driverController.getRawAxis(0) },
        { 0.0 },
        //-driverController.getRawAxis(2)
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

        val levelChooser = SendableChooser<Levels>()

        // levelChooser.addOption("Reset Level", Levels.LEVEL0)
        levelChooser.addOption("Level 1", Levels.LEVEL1)
        levelChooser.addOption("Level 2", Levels.LEVEL2)
        levelChooser.addOption("Level 3", Levels.LEVEL3)
        levelChooser.addOption("Level 4", Levels.LEVEL4)

        SmartDashboard.putData(levelChooser)


        // get selected level with levelChooser.selected



        if (Constants.mode == Constants.States.REAL) {
            //coralIntakeSubsystem = CoralIntakeSubsystem(CoralIntakeIOKraken())
            //algaeIntakeSubsystem = AlgaeIntakeSubsystem(AlgaeIntakeIOKraken())
            //elevatorSubsystem = ElevatorSubsystem(ElevatorIOKraken())
            //climberSubsystem = ClimberSubsystem(ClimberIOKraken())
        }
        else {
            // coralIntakeSubsystem = CoralIntakeSubsystem(CoralIntakeIOSparkMaxSim())

            println("Warning: Simulated subsystems do not exist as no IOClass for them exists!")
            println("Abandon all hope ye who debug here")
        }

        // Because we don't have SimIO Classes
        // I was going to use chatgpt to write a paragraph on why this line of code sucks, but I had to log in
        // So pretend there is a long essay on why it is horrible
        elevatorSubsystem?.let { algaeIntakeSubsystem?.let { it1 -> coralIntakeSubsystem?.let { it2 ->
            climberSubsystem?.let { it3 ->
                initDashboard(it, it1,
                    it2, it3
                )
            }
        } } }

        driverController.L1().onTrue(AlignToAprilTagCommand(driveSubsystem))

        if (elevatorSubsystem != null && coralIntakeSubsystem != null) {
            driverController.R1().onTrue(
                Routines.placeCoralAtLevel(
                    levelChooser.selected.lvl,
                    elevatorSubsystem!!,
                    coralIntakeSubsystem!!
                )
            )
        }

        if (elevatorSubsystem != null && algaeIntakeSubsystem != null) {
            driverController.R2().onTrue(
                Routines.takeAlgaeAtLevel(
                    levelChooser.selected.lvl,
                    elevatorSubsystem!!,
                    algaeIntakeSubsystem!!
                )
            )
        }

        if (coralIntakeSubsystem != null) {
            driverController.L2().onTrue(
                Routines.takeCoral(
                    coralIntakeSubsystem!!,
                )
            )
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
