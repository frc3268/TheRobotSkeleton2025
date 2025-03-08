package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.lib.AutoSequence
import frc.lib.FieldLocation
import frc.lib.FieldPositions
import frc.lib.swerve.SwerveDriveBase
import frc.robot.algaeintake.AlgaeIntakeIOSparkMax
import frc.robot.algaeintake.AlgaeIntakeSubsystem
import frc.robot.climber.ClimberIOKraken
import frc.robot.climber.ClimberSubsystem
import frc.robot.commands.*
import frc.robot.coralintake.CoralIntakeIOSparkMax
import frc.robot.coralintake.CoralIntakeSubsystem
import frc.robot.elevator.ElevatorIOKraken
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

    val driveSubsystem = SwerveDriveBase(Pose2d())



    private val driverController = CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT)

    // There must be a better way to do this! Oh well.
    var coralIntakeSubsystem: CoralIntakeSubsystem? = null
    var elevatorSubsystem: ElevatorSubsystem? = null
    var algaeIntakeSubsystem: AlgaeIntakeSubsystem? = null
    var climberSubsystem: ClimberSubsystem? = null


    val autochooser = SendableChooser<Command>()

    val teleopCommand = SwerveJoystickDrive(
        driveSubsystem,
        { driverController.getRawAxis(1) },
        { driverController.getRawAxis(0) },
        { -driverController.getRawAxis(4) },
        { true }
    )





//    //type is () -> Command because otherwise CommandScheduler complains that each one has already been scheduled
//    val autos: MutableMap<String, () -> Command> = mutableMapOf(
//        "goToProcessor" to { goto(FieldPositions.processor) },
//        "goToSourceLeft" to {goto(FieldPositions.sourceLeft)},
//        "goToSourceRight" to {goto(FieldPositions.sourceRight)},
//        "goToReefLeftClose" to {goto(FieldPositions.reefLeftClose)},
//        "goToReefLeftFar" to {goto(FieldPositions.reefLeftFar)},
//        "goToReefRightClose" to {goto(FieldPositions.reefRightClose)},
//        "goToReefRightFar" to {goto(FieldPositions.reefRightFar)},
//
//        "pickUpCoral" to {Routines.takeCoral(coralIntakeSubsystem!!, elevatorSubsystem!!)},
//        "placeCoralL1" to {Routines.placeCoralAtLevel(
//            Constants.Levels.LEVEL1.lvl,
//            elevatorSubsystem!!,
//            coralIntakeSubsystem!!
//        )},
//        "placeCoralL2" to {Routines.placeCoralAtLevel(
//            Constants.Levels.LEVEL2.lvl,
//            elevatorSubsystem!!,
//            coralIntakeSubsystem!!
//        )},
//        "placeCoralL3" to {Routines.placeCoralAtLevel(
//            Constants.Levels.LEVEL3.lvl,
//            elevatorSubsystem!!,
//            coralIntakeSubsystem!!
//        )},
//        "placeCoralL4" to {Routines.placeCoralAtLevel(
//            Constants.Levels.LEVEL4.lvl,
//            elevatorSubsystem!!,
//            coralIntakeSubsystem!!
//        )},
//        "takeAlgaeL1" to {Routines.takeAlgaeAtLevel(
//            Constants.Levels.LEVEL1.lvl,
//            elevatorSubsystem!!,
//            algaeIntakeSubsystem!!,
//            coralIntakeSubsystem!!
//        )},
//        "takeAlgaeL2" to {Routines.takeAlgaeAtLevel(
//            Constants.Levels.LEVEL2.lvl,
//            elevatorSubsystem!!,
//            algaeIntakeSubsystem!!,
//            coralIntakeSubsystem!!
//        )},
//        "takeAlgaeL3" to {Routines.takeAlgaeAtLevel(
//            Constants.Levels.LEVEL3.lvl,
//            elevatorSubsystem!!,
//            algaeIntakeSubsystem!!,
//            coralIntakeSubsystem!!
//        )},
//        "takeAlgaeL4" to {Routines.takeAlgaeAtLevel(
//            Constants.Levels.LEVEL4.lvl,
//            elevatorSubsystem!!,
//            algaeIntakeSubsystem!!,
//            coralIntakeSubsystem!!
//        )},
//        "dropAlgae" to {algaeIntakeSubsystem!!.dropAlgae()}
//    )

    fun goto(goal: FieldLocation): Command {
        return SwerveAutoDrive(
            {goal},
            driveSubsystem
        )
    }

    
    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {


        val levelChooser = SendableChooser<Constants.Levels>()


        // levelChooser.addOption("Reset Level", Levels.LEVEL0)
        levelChooser.addOption("Level 1", Constants.Levels.LEVEL1)
        levelChooser.addOption("Level 2", Constants.Levels.LEVEL2)
        levelChooser.setDefaultOption("Level 3", Constants.Levels.LEVEL3)
        levelChooser.addOption("Level 4", Constants.Levels.LEVEL4)

        GeneralTab.add(levelChooser)

        val rightChooser = SendableChooser<Boolean>()

        rightChooser.setDefaultOption("left", false)
        rightChooser.addOption("right", true)


        // get selected level with levelChooser.selected
        if (Constants.mode == Constants.States.REAL) {
            coralIntakeSubsystem = CoralIntakeSubsystem(CoralIntakeIOSparkMax())
            //algaeIntakeSubsystem = AlgaeIntakeSubsystem(AlgaeIntakeIOSparkMax())
            elevatorSubsystem = ElevatorSubsystem(ElevatorIOKraken())
            climberSubsystem = ClimberSubsystem(ClimberIOKraken())
        }
        else {
            // coralIntakeSubsystem = CoralIntakeSubsystem(CoralIntakeIOSparkMaxSim())

            println("Warning: Simulated subsystems do not exist as no IOClass for them exists!")
            println("Abandon all hope ye who debug here")
        }

        val rbChooser = SendableChooser<Command>()

        rbChooser.setDefaultOption("Align to April Tag", AlignToAprilTagCommand(driveSubsystem, {rightChooser.selected}))
        rbChooser.addOption("Align to Source Left", goto(FieldPositions.sourceLeft))
        rbChooser.addOption("Align to Source Right", goto(FieldPositions.sourceRight))


        if (elevatorSubsystem != null && coralIntakeSubsystem != null) {

            autochooser.addOption("do nothing", WaitCommand(3.0))

            autochooser.setDefaultOption("taxi", goto(FieldPositions.taxi).andThen(driveSubsystem.zeroHeadingCommand()))

            autochooser.setDefaultOption(
                "go left" ,
                goto(FieldPositions.reefLeftFar).andThen(
                    Routines.placeCoralAtLevel(Constants.Levels.LEVEL3.lvl, elevatorSubsystem!!, coralIntakeSubsystem!!)
                ).andThen(
                    coralIntakeSubsystem!!.outtake().andThen(coralIntakeSubsystem!!.lower()).andThen(coralIntakeSubsystem!!.reset()).andThen(elevatorSubsystem!!.setToPosition(
                        Constants.Levels.LEVEL0.lvl))
                ).andThen(
                    goto(FieldPositions.sourceLeft)
                ).andThen(
                    Routines.takeCoral(coralIntakeSubsystem!!, elevatorSubsystem!!)
                ).andThen(
                    goto(FieldPositions.reefLeftClose)
                ).andThen(
                    Routines.placeCoralAtLevel(Constants.Levels.LEVEL3.lvl, elevatorSubsystem!!, coralIntakeSubsystem!!)
                ).andThen(
                    coralIntakeSubsystem!!.outtake().andThen(coralIntakeSubsystem!!.lower()).andThen(coralIntakeSubsystem!!.reset()).andThen(elevatorSubsystem!!.setToPosition(
                        Constants.Levels.LEVEL0.lvl)))
                    .andThen(
                        goto(FieldPositions.sourceLeft)
                    ).andThen(
                        Routines.takeCoral(coralIntakeSubsystem!!, elevatorSubsystem!!)
                    ).andThen(
                        goto(FieldPositions.reefCenterClose)
                    ).andThen(
                        Routines.placeCoralAtLevel(Constants.Levels.LEVEL3.lvl, elevatorSubsystem!!, coralIntakeSubsystem!!)
                    ).andThen(
                        coralIntakeSubsystem!!.outtake().andThen(coralIntakeSubsystem!!.lower()).andThen(coralIntakeSubsystem!!.reset()).andThen(elevatorSubsystem!!.setToPosition(
                            Constants.Levels.LEVEL0.lvl)))
            )

            autochooser.addOption(
                "go right" ,
                goto(FieldPositions.reefRightFar).andThen(
                    Routines.placeCoralAtLevel(Constants.Levels.LEVEL3.lvl, elevatorSubsystem!!, coralIntakeSubsystem!!)
                ).andThen(
                    coralIntakeSubsystem!!.outtake().andThen(coralIntakeSubsystem!!.lower()).andThen(coralIntakeSubsystem!!.reset()).andThen(elevatorSubsystem!!.setToPosition(
                        Constants.Levels.LEVEL0.lvl))
                ).andThen(
                    goto(FieldPositions.sourceRight)
                ).andThen(
                    Routines.takeCoral(coralIntakeSubsystem!!, elevatorSubsystem!!)
                ).andThen(
                    goto(FieldPositions.reefRightClose)
                ).andThen(
                    Routines.placeCoralAtLevel(Constants.Levels.LEVEL3.lvl, elevatorSubsystem!!, coralIntakeSubsystem!!)
                ).andThen(
                    coralIntakeSubsystem!!.outtake().andThen(coralIntakeSubsystem!!.lower()).andThen(coralIntakeSubsystem!!.reset()).andThen(elevatorSubsystem!!.setToPosition(
                        Constants.Levels.LEVEL0.lvl)))
                    .andThen(
                        goto(FieldPositions.sourceRight)
                    ).andThen(
                        Routines.takeCoral(coralIntakeSubsystem!!, elevatorSubsystem!!)
                    ).andThen(

                        goto(FieldPositions.reefCenterClose)
                    ).andThen(
                        Routines.placeCoralAtLevel(Constants.Levels.LEVEL3.lvl, elevatorSubsystem!!, coralIntakeSubsystem!!)
                    ).andThen(
                        coralIntakeSubsystem!!.outtake().andThen(coralIntakeSubsystem!!.lower()).andThen(coralIntakeSubsystem!!.reset()).andThen(elevatorSubsystem!!.setToPosition(
                            Constants.Levels.LEVEL0.lvl)))
            )

            driverController.leftBumper().onTrue(
                Routines.takeCoral(
                    coralIntakeSubsystem!!, elevatorSubsystem!!
                )
            )

            driverController.leftTrigger().onTrue(
                Routines.placeCoralAtLevel(
                    levelChooser.selected.lvl,
                    elevatorSubsystem!!,
                    coralIntakeSubsystem!!
                )
            )

            driverController.rightBumper().onTrue(
                coralIntakeSubsystem!!.outtake().andThen(coralIntakeSubsystem!!.lower()).andThen(coralIntakeSubsystem!!.reset()).andThen(elevatorSubsystem!!.setToPosition(
                    Constants.Levels.LEVEL0.lvl))
           )
//
//
            driverController.rightTrigger().onTrue(rbChooser.selected)

        }

        driveSubsystem.defaultCommand = teleopCommand

//        GeneralTab
//            .add("drop Algae", algaeIntakeSubsystem?.dropAlgae())
//            .withWidget(BuiltInWidgets.kCommand)
//            .withPosition(3,0)
//            .withSize(1,1)
//
//        GeneralTab
//            .add("Take algae from reef", Routines.takeAlgaeAtLevel(levelChooser.selected.lvl, elevatorSubsystem!!, algaeIntakeSubsystem!!, coralIntakeSubsystem!!))
//            .withWidget(BuiltInWidgets.kComboBoxChooser)
//            .withPosition(2,0)
//            .withSize(2,1)

        GeneralTab
            .add("Autonomous Mode", autochooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1)


//       for (file:File in File(Filesystem.getDeployDirectory().toString() + "/paths").listFiles()?.filter { it.isFile }!!){
//           if(autochooser.selected == null){
//               autochooser.setDefaultOption(file.name,Json.decodeFromStream<AutoSequence>(
//                   file.inputStream()
//                ).toCommandGroup(autos))
//
//            }
//            else {
//               autochooser.addOption(
//                   file.name, Json.decodeFromStream<AutoSequence>(
//                       file.inputStream()
//                    ).toCommandGroup(autos)
//               )
//            }
//       }
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
