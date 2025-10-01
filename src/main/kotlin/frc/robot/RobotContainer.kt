package frc.robot

import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.motorcontrol.Talon
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController


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
//        levelChooser.addOption("Level 1", Constants.Levels.LEVEL1)
//        levelChooser.addOption("Level 2", Constants.Levels.LEVEL2)
//        levelChooser.setDefaultOption("Level 3", Constants.Levels.LEVEL3)
//        levelChooser.addOption("Level 4", Constants.Levels.LEVEL4)
//
//        GeneralTab.add(levelChooser)


        val rightChooser = SendableChooser<Boolean>()

        rightChooser.setDefaultOption("left", false)
        rightChooser.addOption("right", true)


        if (Constants.mode == Constants.States.REAL) {

        } else {

            println("Warning: Simulated subsystems do not exist as no IOClass for them exists!")
            println("Abandon all hope ye who debug here")
        }

        val rbChooser = SendableChooser<Command>()

       rbChooser.setDefaultOption(
           "Align to April Tag",
           AlignToAprilTagCommand(driveSubsystem, { rightChooser.selected })
       )
       rbChooser.addOption("Align to Source Left", goto(FieldPositions.sourceLeft))
       rbChooser.addOption("Align to Source Right", goto(FieldPositions.sourceRight))

       if (Constants.mode == Constants.States.SIM) {
           Shuffleboard.getTab(Constants.TROUBLESHOOTING_TAB)
               .add(AlignToAprilTagCommand(driveSubsystem, { rightChooser.selected }))
       }
       driverController.povDown().onTrue(Routines.inchBack(driveSubsystem))
       driverController.povUp().onTrue(Routines.inchForward(driveSubsystem))
       driverController.povRight().onTrue(Routines.inchRight(driveSubsystem))
       driverController.povLeft().onTrue(Routines.inchLeft(driveSubsystem))
       driveSubsystem.defaultCommand = teleopCommand

    }

    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    val autonomousCommand: Command
        get() {
            return WaitCommand(1.0)
            //fix
            //autochooser.selected
        }
}
