package frc.lib

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.*
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.*
import org.photonvision.EstimatedRobotPose
import java.util.*
import kotlin.math.*

class SwerveDriveBase(startingPose: Pose2d) : SubsystemBase() {
    private val field = Field2d()
    private val shuffleboardTab = Shuffleboard.getTab("Drivetrain")

    private var poseEstimator: SwerveDrivePoseEstimator
    private val modules: List<SwerveModule> =
            SwerveDriveConstants.modules.mapIndexed { _, swerveMod -> SwerveModule(swerveMod) }
    private val gyro = AHRS(SPI.Port.kMXP)

    private var joystickControlledEntry: GenericEntry = shuffleboardTab
            .add("Joystick Control", true)
            .withWidget("Toggle Button")
            .withProperties(mapOf("colorWhenTrue" to "green", "colorWhenFalse" to "maroon"))
            .entry

    private var poseXEntry = shuffleboardTab.add("Pose X", 0.0).entry
    private var poseYEntry = shuffleboardTab.add("Pose Y", 0.0).entry
    private var seesAprilTag = shuffleboardTab.add("Sees April Tag?", false).withWidget(BuiltInWidgets.kBooleanBox).entry

    private var yawOffset: Double = 0.0

    private val camera: Camera


    init {
        SwerveDriveConstants.DrivetrainConsts.thetaPIDController.enableContinuousInput(
                360.0, 0.0
        )
        camera = Camera("hawkeye")
        zeroYaw()
        //https://github.com/Team364/BaseFalconSwerve/issues/8#issuecomment-1384799539
        Timer.delay(1.0)
        resetModulesToAbsolute()
        shuffleboardTab.add("Zero Heading", zeroHeadingCommand()).withWidget(BuiltInWidgets.kCommand)
        shuffleboardTab.add("Robot Heading", gyro).withWidget(BuiltInWidgets.kGyro)




        shuffleboardTab.add(field).withWidget(BuiltInWidgets.kField)

        poseEstimator = SwerveDrivePoseEstimator(SwerveDriveConstants.DrivetrainConsts.kinematics, getYaw(), getModulePositions(), startingPose, VecBuilder.fill(0.1, 0.1, 0.1), VecBuilder.fill(0.5, 0.5, 0.5))
    }

    override fun periodic() {
        //estimate robot pose based on what the encoders say
        poseEstimator.update(getYaw(), getModulePositions())
        //estimate robot pose based on what the camera sees
        seesAprilTag.setBoolean(camera.captureFrame().hasTargets())
        val visionEst: Optional<EstimatedRobotPose>? = camera.getEstimatedPose()
        visionEst?.ifPresent { est ->

            poseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, camera.getEstimationStdDevs(est.estimatedPose.toPose2d()))
        }
        //update module tabs on shuffleboard
        for (mod in modules) {
            mod.updateShuffleboard()
        }
        //update drivetrain tab on shuffleboard
        field.robotPose = getPose()
        poseXEntry.setDouble(getPose().x)
        poseYEntry.setDouble(getPose().y)


    }

    override fun simulationPeriodic() {
        for ((x, state) in constructModuleStatesFromChassisSpeeds(0.0, 0.0, 0.1, true).withIndex()) {
            modules[x].setPointEntry.setDouble(state.angle.degrees)
        }
    }

    private fun zeroYaw() {
        gyro.reset()
    }

    private fun resetModulesToAbsolute() {
        for (mod in modules) {
            mod.resetToAbsolute()
        }
    }

    fun setModuleStates(desiredStates: Array<SwerveModuleState>) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveDriveConstants.DrivetrainConsts.MAX_SPEED_METERS_PER_SECOND)

        for (mod in modules) {
            mod.setDesiredState(desiredStates[mod.moduleConstants.MODULE_NUMBER - 1])
        }
    }

    fun constructModuleStatesFromChassisSpeeds(xSpeedMetersPerSecond: Double, ySpeedMetersPerSecond: Double, turningSpeedDegreesPerSecond: Double, fieldOriented: Boolean): Array<SwerveModuleState> =
            SwerveDriveConstants.DrivetrainConsts.kinematics.toSwerveModuleStates(
                    if (fieldOriented)
                        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, turningSpeedDegreesPerSecond.rotation2dFromDeg().radians, getYaw())
                    else
                        ChassisSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, turningSpeedDegreesPerSecond.rotation2dFromDeg().radians)
            )

    fun stop() {
        for (mod in modules) {
            mod.stop()
        }
    }

    //reset yaw on gyro so that wherever the gyro is pointing is the new forward(0) value
    fun zeroHeadingCommand(): Command {
        return runOnce { zeroYaw() }
    }


    //move robot to pose given in endpose argument
    fun moveToPoseCommand(endPose: Pose2d): Command {
        return run {
            setModuleStates(
                    constructModuleStatesFromChassisSpeeds(
                            -SwerveDriveConstants.DrivetrainConsts.xPIDController.calculate(getPose().x, endPose.x),
                            -SwerveDriveConstants.DrivetrainConsts.yPIDController.calculate(getPose().y, endPose.y),
                            -SwerveDriveConstants.DrivetrainConsts.thetaPIDController.calculate(getYaw().degrees, endPose.rotation.degrees),
                            true
                    ))
        }.until { abs(getPose().translation.getDistance(endPose.translation)) < 0.02 && abs(getYaw().degrees - endPose.rotation.degrees) < 1.5 }
    }

    //getters
    private fun getYaw(): Rotation2d = -(gyro.rotation2d.degrees + yawOffset).IEEErem(360.0).rotation2dFromDeg()
    fun getPitch(): Rotation2d = gyro.pitch.toDouble().rotation2dFromDeg()
    fun getPose(): Pose2d = Pose2d(poseEstimator.estimatedPosition.x, poseEstimator.estimatedPosition.y, poseEstimator.estimatedPosition.rotation)
    fun getModuleStates(): Array<SwerveModuleState> = modules.map { it.getState() }.toTypedArray()
    private fun getModulePositions(): Array<SwerveModulePosition> = modules.map { it.getPosition() }.toTypedArray()
}
