package frc.lib.basics

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.lib.constants.SwerveDriveConstants
import frc.lib.utils.rotation2dFromDeg

//TODO:Maybe a drive function
class SwerveDriveBase(startingPose: Pose2d) : SubsystemBase() {
    private val ShuffleboardTab = Shuffleboard.getTab("Drivetrain")
    val poseEstimator: SwerveDrivePoseEstimator
    private val modules: List<SwerveModule> =
        SwerveDriveConstants.modules.list.mapIndexed { _, swerveMod -> SwerveModule(swerveMod) }
    private val gyro: AHRS = AHRS(SPI.Port.kMXP)
    val kinematics: SwerveDriveKinematics =
        //front left, front right, back left, back right
        //assuming that 0,0 is the center of the robot, and (+,+) means (left, front)
        SwerveDriveKinematics(
            Translation2d(SwerveDriveConstants.DrivetrainConsts.WHEEL_BASE_METERS / 2.0, -SwerveDriveConstants.DrivetrainConsts.TRACK_WIDTH_METERS / 2.0),
            Translation2d(SwerveDriveConstants.DrivetrainConsts.WHEEL_BASE_METERS / 2.0, SwerveDriveConstants.DrivetrainConsts.TRACK_WIDTH_METERS / 2.0),
            Translation2d(-SwerveDriveConstants.DrivetrainConsts.WHEEL_BASE_METERS / 2.0, -SwerveDriveConstants.DrivetrainConsts.TRACK_WIDTH_METERS / 2.0),
            Translation2d(-SwerveDriveConstants.DrivetrainConsts.WHEEL_BASE_METERS / 2.0, SwerveDriveConstants.DrivetrainConsts.TRACK_WIDTH_METERS / 2.0)
        )
    init {
        gyro.calibrate()
        zeroYaw()
        poseEstimator = SwerveDrivePoseEstimator(kinematics, getYaw(), getModulePositions(), startingPose)
        //https://github.com/Team364/BaseFalconSwerve/issues/8#issuecomment-1384799539
        Timer.delay(1.0)
        resetModulesToAbsolute()
        //ShuffleboardTab.add("Robot Heading", gyro)
        //ShuffleboardTab.add("Stop", stopCommand())
    }

    override fun periodic() {
        for (mod in modules){
            mod.updateDashboard()
        }
    }

    override fun simulationPeriodic() {
        var x = 0
        for (state in constructStates(-1.0, 0.0,0.0, true)){
            modules[x].setPointEntry.setDouble(state.angle.degrees)
            x++;
        }
    }

    private fun zeroYaw() {
        gyro.zeroYaw()
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
    fun constructStates(xSpeedMetersPerSecond:Double, ySpeedMetersPerSecond:Double, turningSpeedDegreesPerSecond:Double, fieldOriented:Boolean) : Array<SwerveModuleState> =
        kinematics.toSwerveModuleStates (
            if (fieldOriented)
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSecond,ySpeedMetersPerSecond,turningSpeedDegreesPerSecond.rotation2dFromDeg().radians,getYaw())
            else
                ChassisSpeeds(xSpeedMetersPerSecond,ySpeedMetersPerSecond,turningSpeedDegreesPerSecond))
    fun stop(){
        for(mod in modules){
            mod.stop()
        }
    }

    
    fun stopCommand() :Command{
        return InstantCommand({stop()})
    }
    fun getYaw(): Rotation2d = (gyro.rotation2d.degrees).rotation2dFromDeg()
    fun getPitch(): Rotation2d = gyro.pitch.toDouble().rotation2dFromDeg()
    fun getPose():Pose2d = poseEstimator.estimatedPosition
    fun getModuleStates(): Array<SwerveModuleState> = modules.map { it.getState() }.toTypedArray()
    fun getModulePositions(): Array<SwerveModulePosition> = modules.map { it.getPosition() }.toTypedArray()

}
