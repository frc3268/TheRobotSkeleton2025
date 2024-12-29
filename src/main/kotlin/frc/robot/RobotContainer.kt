package frc.robot

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
import frc.robot.commands.*
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
        "gotoSpeaker" to { goToSpeakerCloser() },
        "gotoSpeakerCenter" to {goto(FieldPositions.speakerCenter)},
        "gotoSpeakerRight" to {goto(FieldPositions.speakerRight)},
        "gotoSpeakerLeft" to {goto(FieldPositions.speakerLeft)},
        "gotoAmp" to {goto(FieldPositions.amp)},
        "goToSourceCloserToBaseline" to {goto(FieldPositions.sourceBaseline)},
        "goToSourceFurtherFromBaseline" to {goto(FieldPositions.sourceNotBaseline)},
        "goToRing" to {WaitCommand(1.0)},
        //todo: other rings
    )

    fun goto(goal: FieldLocation): Command {
        val color = DriverStation.getAlliance()
        val to =
            if (color.isPresent && color.get() == DriverStation.Alliance.Red)
                goal.red
            else
                goal.blue
        return gotoPose(to)
    }

    /*
    OK, why is this here?
    Because this relies on recursion
    since we are evaluating the conditions at runtime, we need to break it all down
    this isn't possible with goto because you pass in a FieldLocation, but midpoint has no fieldlocation
    therefore, goToPose
    as for the weird variable names... talk to matthew, and maybe he'll fix them later

     */
    fun gotoPose(to: Pose2d):Command {
        val toRun = AtomicReference(SequentialCommandGroup())
         return runOnce({
            val points = pathfind(driveSubsystem.getPose(), to)
             val sqi = toRun.get()
             for (point in points){
                 sqi.addCommands(SwerveAutoDrive(
                     {point},
                     Pose2d(0.1,0.1,10.0.rotation2dFromDeg()),
                     driveSubsystem,
                     { driverController.getRawAxis(1) },
                     { -driverController.getRawAxis(0) },
                     { -driverController.getRawAxis(2) }
                 ))
             }
             toRun.set(sqi)
        }, driveSubsystem).andThen(
            toRun.get()
        )
    }

    fun pathfind(from:Pose2d, to:Pose2d):List<Pose2d> {
        val pose = from
        val m: Double = (to.y - pose.y) / (to.x - pose.x)
        val b: Double = -m*to.x + to.y
        for (obstacle in obstacles) {
            val ntwo = b - obstacle.location.y
            val obx = obstacle.location.x
            val btwo = -obx * 2 + ntwo * 2 * m
            val a = m.pow(2) + 1
            val c = ntwo.pow(2) + obx.pow(2) - obstacle.radiusMeters.pow(2)
            val det = btwo.pow(2) - 4 * a * c
            if (det >= 0) {
                val intersection: Pose2d = if (pose.x > obstacle.location.x) Pose2d(
                    ((-btwo + sqrt(det)) / (2 * a)), (m * ((-btwo + sqrt(det)) / (2 * a)) + b), pose.rotation
                ) else Pose2d(
                    ((-btwo - sqrt(det)) / (2 * a)), (m * ((-btwo + sqrt(det)) / (2 * a)) + b), pose.rotation
                )
                if(intersection.x in pose.x..to.x || intersection.x in to.x..pose.x) {
                    val midpoint =
                        Pose2d(
                            //FIX THIS SO IT DOESNt COLLIDE WITH WALLS
                            intersection.x + SwerveDriveConstants.DrivetrainConsts.TRACK_WIDTH_METERS + obstacle.radiusMeters,
                            -1 / m * (intersection.x + SwerveDriveConstants.DrivetrainConsts.TRACK_WIDTH_METERS + obstacle.radiusMeters),
                            pose.rotation
                        )
                    driveSubsystem.field.getObject(midpoint.x.toString()).pose = intersection
                    return listOf(pathfind(from, midpoint), pathfind(midpoint, to)).flatten()

                }
            }
        }
        return listOf(to)
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


    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
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
