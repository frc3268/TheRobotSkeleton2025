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
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.lib.constants.SwerveDriveConstants
import frc.lib.utils.rotation2dFromDeg

//TODO:Maybe a drive function
class SwerveDriveBase(startingPose: Pose2d) : SubsystemBase() {
    private val ShuffleboardTab = Shuffleboard.getTab("Drivetrain")
    private val ShufflebooardEntries: List<GenericEntry> = listOf(
        ShuffleboardTab.add("Mod 1 Angle Encoder", 0.0).entry,
        ShuffleboardTab.add("Mod 2 Angle Encoder", 0.0).entry,
        ShuffleboardTab.add("Mod 3 Angle Encoder", 0.0).entry,
        ShuffleboardTab.add("Mod 4 Angle Encoder", 0.0).entry
    )
    val poseEstimator: SwerveDrivePoseEstimator
    private val modules: List<SwerveModule> =
        SwerveDriveConstants.modules.list.mapIndexed { _, swerveMod -> SwerveModule(swerveMod) }
    private val gyro: AHRS = AHRS(SPI.Port.kMXP)
    val kinematics: SwerveDriveKinematics =
        SwerveDriveKinematics(
            Translation2d(SwerveDriveConstants.DrivetrainConsts.WHEEL_BASE_METERS / 2.0, SwerveDriveConstants.DrivetrainConsts.TRACK_WIDTH_METERS / 2.0),
            Translation2d(SwerveDriveConstants.DrivetrainConsts.WHEEL_BASE_METERS / 2.0, -SwerveDriveConstants.DrivetrainConsts.TRACK_WIDTH_METERS / 2.0),
            Translation2d(-SwerveDriveConstants.DrivetrainConsts.WHEEL_BASE_METERS / 2.0, SwerveDriveConstants.DrivetrainConsts.TRACK_WIDTH_METERS / 2.0),
            Translation2d(-SwerveDriveConstants.DrivetrainConsts.WHEEL_BASE_METERS / 2.0, -SwerveDriveConstants.DrivetrainConsts.TRACK_WIDTH_METERS / 2.0)
        )
    init {
        gyro.calibrate()
        zeroYaw()
        poseEstimator = SwerveDrivePoseEstimator(kinematics, getYaw(), getModulePositions(), startingPose)
        //https://github.com/Team364/BaseFalconSwerve/issues/8#issuecomment-1384799539
        Timer.delay(1.0)
        resetModulesToAbsolute()
    }

    override fun periodic() {
        for (mod in modules){
            ShufflebooardEntries[mod.moduleConstants.MODULE_NUMBER - 1].setDouble(mod.getPosition().angle.degrees)
        }
    }

    override fun simulationPeriodic() {
        var x = 0
        for (state in constructStates(-0.5, 0.0,0.0, false)){
            ShufflebooardEntries[x].setDouble(state.speedMetersPerSecond)
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
                ChassisSpeeds(xSpeedMetersPerSecond,ySpeedMetersPerSecond,turningSpeedDegreesPerSecond.rotation2dFromDeg().radians))
    fun stop(){
        for(mod in modules){
            mod.stop()
        }
    }
    fun getYaw(): Rotation2d = gyro.rotation2d
    fun getPitch(): Rotation2d = gyro.pitch.toDouble().rotation2dFromDeg()
    fun getPose():Pose2d = poseEstimator.estimatedPosition
    fun getModuleStates(): Array<SwerveModuleState> = modules.map { it.getState() }.toTypedArray()
    fun getModulePositions(): Array<SwerveModulePosition> = modules.map { it.getPosition() }.toTypedArray()

}
