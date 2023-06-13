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
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.lib.constants.SwerveDriveConstants


class SwerveDriveBase(startingPose: Pose2d) : SubsystemBase() {

    //swerve is odom required camera optional
    private val poseEstimator: SwerveDrivePoseEstimator
    private val modules: List<SwerveModule> =
        SwerveDriveConstants.modules.list.mapIndexed { _, swerveMod -> SwerveModule(swerveMod) }
    private val gyro: AHRS = AHRS(SPI.Port.kMXP)
    private val kinematics: SwerveDriveKinematics =
        SwerveDriveKinematics(
            Translation2d(SwerveDriveConstants.DrivetrainConsts.WHEEL_BASE_METERS / 2.0, SwerveDriveConstants.DrivetrainConsts.TRACK_WIDTH_METERS / 2.0),
            Translation2d(SwerveDriveConstants.DrivetrainConsts.WHEEL_BASE_METERS / 2.0, -SwerveDriveConstants.DrivetrainConsts.TRACK_WIDTH_METERS / 2.0),
            Translation2d(-SwerveDriveConstants.DrivetrainConsts.WHEEL_BASE_METERS / 2.0, SwerveDriveConstants.DrivetrainConsts.TRACK_WIDTH_METERS / 2.0),
            Translation2d(-SwerveDriveConstants.DrivetrainConsts.WHEEL_BASE_METERS / 2.0, -SwerveDriveConstants.DrivetrainConsts.TRACK_WIDTH_METERS / 2.0)
        )

    init {
        gyro.calibrate()
        zeroGyro()
        poseEstimator = SwerveDrivePoseEstimator(kinematics, getYaw(), getModulePositions(), startingPose)
        Timer.delay(1.0)
        resetModulesToAbsolute()
    }

    override fun periodic() {
        poseEstimator.update(getYaw(), getModulePositions())
        for (mod in modules) {
            SmartDashboard.putNumber("Mod " + mod.moduleConstants.MODULE_NUMBER + " Cancoder", mod.getEncoderMeasurement().degrees)
            SmartDashboard.putNumber("Mod " + mod.moduleConstants.MODULE_NUMBER + " Integrated", mod.getPosition().angle.degrees)
            SmartDashboard.putNumber("Mod " + mod.moduleConstants.MODULE_NUMBER + " Velocity", mod.getState().speedMetersPerSecond)
        }
    }

    /** This method will be called once per scheduler run during simulation  */
    override fun simulationPeriodic() {
    }

    fun drive(translation: Translation2d, rotation: Double, isOpenLoop: Boolean, fieldOriented: Boolean) {
        val swerveModuleStates: Array<SwerveModuleState> = kinematics.toSwerveModuleStates(
            if (fieldOriented) {
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation,
                    getYaw()
                )
            } else {
                ChassisSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation
                )
            }
        )

        setModuleStates(swerveModuleStates, isOpenLoop)
    }

    //--gyro--//
    fun getYaw(): Rotation2d {
        return gyro.rotation2d
    }

    fun getPitch(): Rotation2d {
        return Rotation2d.fromDegrees(gyro.pitch.toDouble())
    }

    fun zeroGyro() {
        gyro.zeroYaw()
    }

    fun getPose(): Pose2d {
        return poseEstimator.getEstimatedPosition()
    }

    //--odometry--//
    fun resetOdometry(pose: Pose2d) {
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose)
    }

    //--modules--//
    fun getModuleStates(): Array<SwerveModuleState> {
        return modules.map { it.getState() }
            .toTypedArray()
    }

    fun getModulePositions(): Array<SwerveModulePosition> {
        return modules.map { it.getPosition() }
            .toTypedArray()
    }

    fun resetModulesToAbsolute() {
        for (mod in modules) {
            mod.resetToAbsolute()
        }
    }

    fun setModuleStates(desiredStates: Array<SwerveModuleState>, isOpenLoop: Boolean) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveDriveConstants.DrivetrainConsts.MAX_SPEED_METERS_PER_SECOND)

        for (mod in modules) {
            mod.setDesiredState(desiredStates[mod.moduleConstants.MODULE_NUMBER -1], isOpenLoop)
        }
    }
}
