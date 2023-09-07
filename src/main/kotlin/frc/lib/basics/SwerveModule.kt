package frc.lib.basics

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.AnalogEncoder
import frc.lib.constants.SwerveDriveConstants
import frc.lib.utils.rotation2dFromDeg
import kotlin.math.abs

/*
Props: drive motor, drive encoder, angle motor, angle encoder, absolute encoder
Get: Cancoder measurement, Module state(velocity) and position
Set: Module state
 */
class SwerveModule(val moduleConstants: SwerveDriveConstants.ModuleConstants) {
    private val driveMotor:CANSparkMax = CANSparkMax(moduleConstants.DRIVE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val angleMotor:CANSparkMax = CANSparkMax(moduleConstants.ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)

    private val driveEncoder:RelativeEncoder = driveMotor.encoder
    private val angleEncoder:RelativeEncoder = angleMotor.encoder

    private val absoluteEncoder:AnalogEncoder = AnalogEncoder(moduleConstants.ENCODER_ID)

    init {
        absoluteEncoder.distancePerRotation = SwerveDriveConstants.EncoderConsts.POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION
        absoluteEncoder.positionOffset = moduleConstants.ANGLE_OFFSET.degrees
        driveEncoder.positionConversionFactor = SwerveDriveConstants.DriveMotorConsts.POSITION_CONVERSION_FACTOR_METERS_PER_ROTATION
        driveEncoder.velocityConversionFactor = SwerveDriveConstants.DriveMotorConsts.VELOCITY_CONVERSION_FACTOR_METERS_PER_SECOND
        angleEncoder.positionConversionFactor = SwerveDriveConstants.AngleMotorConsts.POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION
        resetToAbsolute()

        driveMotor.inverted = moduleConstants.DRIVE_MOTOR_REVERSED
        angleMotor.inverted = moduleConstants.ANGLE_MOTOR_REVERSED

        driveMotor.setOpenLoopRampRate(0.9)
        angleMotor.setOpenLoopRampRate(0.9)

        SwerveDriveConstants.DrivetrainConsts.turnController.enableContinuousInput(
            -180.0,180.0
        )
    }

    fun resetToAbsolute(){
        driveEncoder.position = 0.0
        angleEncoder.position = getAbsoluteEncoderMeasurement().degrees
    }
    fun getAbsoluteEncoderMeasurement() : Rotation2d = ((absoluteEncoder.absolutePosition * 360.0) + moduleConstants.ANGLE_OFFSET.degrees).rotation2dFromDeg()
    fun getState() : SwerveModuleState = SwerveModuleState(driveEncoder.velocity, angleEncoder.position.rotation2dFromDeg())
    fun getPosition() : SwerveModulePosition = SwerveModulePosition(driveEncoder.position, angleEncoder.position.rotation2dFromDeg())

    fun setDesiredState(desiredState:SwerveModuleState){
        if (abs(desiredState.speedMetersPerSecond) < 0.001){
            stop()
            return
        }
        val optimizedState = SwerveModuleState.optimize(desiredState, getState().angle)
        //TODO: 5.0 should be a const
        driveMotor.set(optimizedState.speedMetersPerSecond / 5.0)
        angleMotor.set(SwerveDriveConstants.DrivetrainConsts.turnController.calculate(getState().angle.degrees, optimizedState.angle.degrees))
    }
    fun stop(){
        driveMotor.set(0.0)
        angleMotor.set(0.0)
    }

}