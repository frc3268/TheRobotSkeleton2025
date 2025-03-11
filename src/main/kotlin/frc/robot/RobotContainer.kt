package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.lib.FieldLocation
import frc.lib.FieldPositions
import frc.lib.swerve.SwerveDriveBase
import frc.robot.algaeintake.AlgaeIntakeIOSparkMax
import frc.robot.algaeintake.AlgaeIntakeSubsystem
import frc.robot.climber.ClimberIOSparkMax
import frc.robot.climber.ClimberSubsystem
import frc.robot.commands.AlignToAprilTagCommand
import frc.robot.commands.Routines
import frc.robot.commands.SwerveAutoDrive
import frc.robot.commands.SwerveJoystickDrive
import frc.robot.coralintake.CoralIntakeSubsystem
import frc.robot.elevator.ElevatorIOKraken
import frc.robot.elevator.ElevatorSubsystem

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    private val GeneralTab = Shuffleboard.getTab("General")
    private val CalibrationTab = Shuffleboard.getTab(Constants.CALIBRATION_TAB)
    val elevatorHeightDesiredEntry = CalibrationTab.add("Desired Elevator Height", 0.0).withWidget(BuiltInWidgets.kNumberSlider).entry



    val driveSubsystem = SwerveDriveBase(Pose2d())



    private val driverController = CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT)

    // There must be a better way to do this! Oh well.
    var coralIntakeSubsystem: CoralIntakeSubsystem? = null
    var elevatorSubsystem: ElevatorSubsystem? = null
    var algaeIntakeSubsystem: AlgaeIntakeSubsystem? = null
    var climberSubsystem: ClimberSubsystem? = null

    var elevatorHeight: Double = 0.0


    val autochooser = SendableChooser<Command>()

    val teleopCommand = SwerveJoystickDrive(
        driveSubsystem,
        { driverController.getRawAxis(1) },
        { driverController.getRawAxis(0) },
        { -driverController.getRawAxis(4) },
        { true }
    )


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
            //coralIntakeSubsystem = CoralIntakeSubsystem(CoralIntakeIOSparkMax())
            algaeIntakeSubsystem = AlgaeIntakeSubsystem(AlgaeIntakeIOSparkMax())
            elevatorSubsystem = ElevatorSubsystem(ElevatorIOKraken())
            climberSubsystem = ClimberSubsystem(ClimberIOSparkMax())
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

            autochooser.setDefaultOption("taxi", SwerveJoystickDrive(driveSubsystem, {1.0}, {0.0}, {0.0},{false} ).withTimeout(1.0))

            autochooser.addOption(
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

            //just lower the elevator little bro
            driverController.y().onTrue(
                coralIntakeSubsystem!!.stopIntake().
                andThen(coralIntakeSubsystem!!.lower().alongWith(elevatorSubsystem!!.setToPosition(0.0)))
            )

            driverController.povDown().onTrue(Routines.inchBack(driveSubsystem))
            driverController.povUp().onTrue(Routines.inchForward(driveSubsystem))
            driverController.povRight().onTrue(Routines.inchRight(driveSubsystem))
            driverController.povLeft().onTrue(Routines.inchLeft(driveSubsystem))

            driverController.rightTrigger().onTrue(elevatorSubsystem!!.setToPosition(levelChooser.selected.lvl))
            GeneralTab.add("0",elevatorSubsystem!!.setToPosition(Constants.Levels.LEVEL0.lvl) )
            GeneralTab.add("1",elevatorSubsystem!!.setToPosition(Constants.Levels.LEVEL1.lvl) )
            GeneralTab.add("2",elevatorSubsystem!!.setToPosition(Constants.Levels.LEVEL2.lvl) )
            GeneralTab.add("3",elevatorSubsystem!!.setToPosition(Constants.Levels.LEVEL3.lvl) )

            CalibrationTab.add(elevatorSubsystem!!.setToPosition(elevatorHeightDesiredEntry.getDouble(Constants.Levels.LEVEL0.lvl)))

        }


        driveSubsystem.defaultCommand = teleopCommand

        GeneralTab
            .add("Autonomous Mode", autochooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1)
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
