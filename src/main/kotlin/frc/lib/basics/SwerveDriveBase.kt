package frc.lib.basics

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.*
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.*
import frc.lib.constants.SwerveDriveConstants
import frc.lib.utils.*
import org.photonvision.EstimatedRobotPose
import java.util.*
import kotlin.math.*

class SwerveDriveBase(var startingPose: Pose2d) : SubsystemBase() {
    val field = Field2d()
    private val ShuffleboardTab = Shuffleboard.getTab("Dirvetrain")

    var poseEstimator: SwerveDrivePoseEstimator
    val camera:Camera
    private val modules: List<SwerveModule> =
        SwerveDriveConstants.modules.mapIndexed { _, swerveMod -> SwerveModule(swerveMod) }
    private val gyro = AHRS(SPI.Port.kMXP)

    private var joystickControlledEntry: GenericEntry = ShuffleboardTab
        .add("Joystick Control", true)
        .withWidget("Toggle Button")
        .withProperties(mapOf("colorWhenTrue" to "green", "colorWhenFalse" to "maroon"))
        .getEntry()

    private var poseXEntry = ShuffleboardTab.add("Pose X", 0.0).entry

    private var poseYEntry = ShuffleboardTab.add("Pose Y", 0.0).entry

    var yawOffset:Double = 0.0
    init {
        DriverStation.getAlliance().ifPresent {
            color ->
            if(color == DriverStation.Alliance.Blue){
                yawOffset = 180.0
            }
        }


        gyro.reset()
       //https://github.com/Team364/BaseFalconSwerve/issues/8#issuecomment-1384799539
        Timer.delay(1.0)
        zeroYaw()
        resetModulesToAbsolute()
        ShuffleboardTab.add("Stop", stopCommand()).withWidget(BuiltInWidgets.kCommand)
        ShuffleboardTab.add("Zero Heading", zeroHeadingCommand()).withWidget(BuiltInWidgets.kCommand)
        ShuffleboardTab.add("Dig In", digInCommand()).withWidget(BuiltInWidgets.kCommand)
        ShuffleboardTab.add("Robot Heading", gyro).withWidget(BuiltInWidgets.kGyro)

        camera = Camera("hawkeye", "")
        ShuffleboardTab.add(field).withWidget(BuiltInWidgets.kField)
        val visionEst: Optional<EstimatedRobotPose>? = camera.getEstimatedPose()
        visionEst?.ifPresent { est ->
            startingPose = est.estimatedPose.toPose2d()
        }
        poseEstimator = SwerveDrivePoseEstimator(SwerveDriveConstants.DrivetrainConsts.kinematics, getYaw(), getModulePositions(), startingPose, VecBuilder.fill(0.1, 0.1, 0.1),  VecBuilder.fill(0.5, 0.5, 0.5))
    }

    override fun periodic() {
        poseEstimator.update(getYaw(), getModulePositions())
        for (mod in modules){
            mod.updateShuffleboard()
        }
        //estimate robot pose based on what the camera sees
        val visionEst: Optional<EstimatedRobotPose>? = camera.getEstimatedPose()
        visionEst?.ifPresent { est ->
            poseEstimator.addVisionMeasurement(
                est.estimatedPose.toPose2d(), est.timestampSeconds, camera.getEstimationStdDevs(est.estimatedPose.toPose2d())
            )
            System.out.println(poseEstimator.estimatedPosition.x)
            System.out.println(poseEstimator.estimatedPosition.y)
        }
        field.robotPose = getPose()
        poseXEntry.setDouble(getPose().x)
        poseYEntry.setDouble(getPose().y)


    }

    override fun simulationPeriodic() {
        for ((x, state) in constructModuleStatesFromChassisSpeeds(0.0, 0.0,0.1, true).withIndex()){
            modules[x].setPointEntry.setDouble(state.angle.degrees)
        }
    }


    fun zeroYaw() {
        gyro.reset()
    }
    private fun resetModulesToAbsolute(){
        for (mod in modules){
            mod.resetToAbsolute()
        }
    }

    fun setModuleStates(desiredStates: Array<SwerveModuleState>) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveDriveConstants.DrivetrainConsts.MAX_SPEED_METERS_PER_SECOND)

        for (mod in modules) {
            mod.setDesiredState(desiredStates[mod.moduleConstants.MODULE_NUMBER - 1])
        }
    }

    fun constructModuleStatesFromChassisSpeeds(xSpeedMetersPerSecond:Double, ySpeedMetersPerSecond:Double, turningSpeedDegreesPerSecond:Double, fieldOriented:Boolean) : Array<SwerveModuleState> =
        SwerveDriveConstants.DrivetrainConsts.kinematics.toSwerveModuleStates (
            if (fieldOriented)
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSecond,ySpeedMetersPerSecond,turningSpeedDegreesPerSecond.rotation2dFromDeg().radians,getYaw())
            else
                ChassisSpeeds(xSpeedMetersPerSecond,ySpeedMetersPerSecond,turningSpeedDegreesPerSecond.rotation2dFromDeg().radians)
        )
    fun stop(){
        for(mod in modules){
            mod.stop()
        }
    }

    fun stopCommand() : Command {
        return run {stop()}.until { joystickControlledEntry.getBoolean(true) }.beforeStarting (runOnce{joystickControlledEntry.setBoolean(false)} )
    }


     fun digInCommand(): Command{
       return runOnce{
            joystickControlledEntry.setBoolean(false)
            //because driving can be field-oriented, we need to take an angle perprendicular to the direction of the wheels, so that motion stops if we are being pushed
            }.andThen(
            run{
                setModuleStates(
                    arrayOf(SwerveModuleState(0.01, 45.0.rotation2dFromDeg()),
                            SwerveModuleState(0.01, (-45.0).rotation2dFromDeg()),
                            SwerveModuleState(0.01, 45.0.rotation2dFromDeg()),
                            SwerveModuleState(0.01, (-45.0).rotation2dFromDeg())
                ))
            }.until {joystickControlledEntry.getBoolean(true)})
    }

    fun zeroHeadingCommand(): Command {
        // Inline construction of command goes here.
        // runOnce implicitly requires this subsystem.
        return runOnce { zeroYaw() }
    }


    fun robotPoseToBCommand(endPose: Pose2d): Command{
        SwerveDriveConstants.DrivetrainConsts.thetaPIDController.enableContinuousInput(
            360.0, 0.0
        )
        return run {
            setModuleStates(
                constructModuleStatesFromChassisSpeeds(
                -SwerveDriveConstants.DrivetrainConsts.xPIDController.calculate(getPose().x,  endPose.x),
                SwerveDriveConstants.DrivetrainConsts.yPIDController.calculate(getPose().y,  endPose.y),
                SwerveDriveConstants.DrivetrainConsts.thetaPIDController.calculate(getYaw().degrees,  endPose.rotation.degrees),
                true
            ))
        }.until { abs(getPose().translation.getDistance(endPose.translation)) < 0.05 && abs(getYaw().degrees - endPose.rotation.degrees) < 1.5 }
    }

    fun getYaw(): Rotation2d = (gyro.rotation2d.degrees + yawOffset).IEEErem(360.0).rotation2dFromDeg()
    fun getPitch(): Rotation2d = gyro.pitch.toDouble().rotation2dFromDeg()
    fun getPose():Pose2d = Pose2d(-poseEstimator.estimatedPosition.x, poseEstimator.estimatedPosition.y, poseEstimator.estimatedPosition.rotation)
    fun getModuleStates(): Array<SwerveModuleState> = modules.map { it.getState() }.toTypedArray()
    fun getModulePositions(): Array<SwerveModulePosition> = modules.map { it.getPosition() }.toTypedArray()

    fun zeroPoseToFieldPositionCommand(startingPose: Pose2d){
            resetModulesToAbsolute()
            poseEstimator.resetPosition(getYaw(), getModulePositions(), startingPose)
    }

}
