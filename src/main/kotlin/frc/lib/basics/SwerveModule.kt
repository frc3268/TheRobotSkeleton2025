package frc.lib.basics

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.RelativeEncoder
import com.revrobotics.SparkMaxPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.AnalogEncoder
import frc.lib.constants.SwerveDriveConstants

class SwerveModule(val moduleConstants: SwerveDriveConstants.ModuleConstants) {
    //encoder
    private val encoder: AnalogEncoder = AnalogEncoder(moduleConstants.ENCODER_ID)

    //angle motor
    private val angleMotor: CANSparkMax = CANSparkMax(moduleConstants.ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val integratedAngleEncoder: RelativeEncoder = angleMotor.getEncoder()

    private val angleController: SparkMaxPIDController = angleMotor.getPIDController()

    //drive motor
    private val driveMotor: CANSparkMax = CANSparkMax(moduleConstants.DRIVE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val integratedDriveEncoder: RelativeEncoder = driveMotor.encoder
    private val driveController: SparkMaxPIDController = driveMotor.getPIDController()

    //misc
    var lastAngle: Rotation2d


    private val feedforward: SimpleMotorFeedforward =
        SimpleMotorFeedforward(
            SwerveDriveConstants.ModuleGains.DRIVE_KS,
            SwerveDriveConstants.ModuleGains.DRIVE_KV,
            SwerveDriveConstants.ModuleGains.DRIVE_KA
        )

    init {
        //config everything
        configEncoder()
        lastAngle = getState().angle
        configAngleMotor()
        configDriveMotor()
    }

    fun configDriveMotor() {
        driveMotor.restoreFactoryDefaults()
        driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20)
        driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20)
        driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 50)
        driveMotor.setSmartCurrentLimit(SwerveDriveConstants.DriveMotorConsts.CONTINUOUS_CURRENT_LIMIT)
        driveMotor.setInverted(SwerveDriveConstants.DriveMotorConsts.INVERT)
        driveMotor.setIdleMode(SwerveDriveConstants.DriveMotorConsts.NEUTRAL_MODE)
        integratedDriveEncoder.setVelocityConversionFactor(SwerveDriveConstants.DriveMotorConsts.VELOCITY_CONVERSION_FACTOR_METERS_PER_SECOND)
        integratedDriveEncoder.setPositionConversionFactor(SwerveDriveConstants.DriveMotorConsts.POSITION_CONVERSION_FACTOR_METERS_PER_ROTATION)
        driveController.setP(SwerveDriveConstants.DriveMotorConsts.KP)
        driveController.setI(SwerveDriveConstants.DriveMotorConsts.KI)
        driveController.setD(SwerveDriveConstants.DriveMotorConsts.KD)
        driveController.setFF(SwerveDriveConstants.DriveMotorConsts.KFF)
        driveMotor.enableVoltageCompensation(SwerveDriveConstants.DrivetrainConsts.VOLTAGE_COMPENSATION)
        driveMotor.burnFlash()
        integratedDriveEncoder.setPosition(0.0)
    }

    fun configAngleMotor() {
        angleMotor.restoreFactoryDefaults()
        angleMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 500)
        angleMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20)
        angleMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 500)
        angleMotor.setSmartCurrentLimit(SwerveDriveConstants.AngleMotorConsts.CONTINUOUS_CURRENT_LIMIT)
        angleMotor.setInverted(SwerveDriveConstants.AngleMotorConsts.INVERT)
        angleMotor.setIdleMode(SwerveDriveConstants.AngleMotorConsts.NEUTRAL_MODE)
        angleController.setP(SwerveDriveConstants.AngleMotorConsts.KP)
        angleController.setI(SwerveDriveConstants.AngleMotorConsts.KI)
        angleController.setD(SwerveDriveConstants.AngleMotorConsts.KD)
        angleController.setFF(SwerveDriveConstants.AngleMotorConsts.KFF)
        angleMotor.enableVoltageCompensation(SwerveDriveConstants.DrivetrainConsts.VOLTAGE_COMPENSATION)
        angleMotor.burnFlash()
        resetToAbsolute()
    }


    fun configEncoder() {
        //debug below
        encoder.distancePerRotation = SwerveDriveConstants.EncoderConsts.POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION
        encoder.positionOffset = moduleConstants.ANGLE_OFFSET.degrees
    }

    fun setDesiredState(desiredState: SwerveModuleState, isOpenLoop: Boolean) {
        val optimizedState = SwerveModuleState.optimize(desiredState, getEncoderMeasurement())
        if (isOpenLoop) {
            driveMotor.set(optimizedState.speedMetersPerSecond / SwerveDriveConstants.DrivetrainConsts.MAX_SPEED_METERS_PER_SECOND)
        } else {
            driveController.setReference(
                desiredState.speedMetersPerSecond,
                CANSparkMax.ControlType.kVelocity,
                0,
                feedforward.calculate(optimizedState.speedMetersPerSecond))
        }
        val angle: Rotation2d = if (Math.abs(optimizedState.speedMetersPerSecond) <= (SwerveDriveConstants.DrivetrainConsts.MAX_SPEED_METERS_PER_SECOND * 0.01)) {
            lastAngle
        } else {
            optimizedState.angle
        }
        angleController.setReference(angle.degrees, CANSparkMax.ControlType.kPosition)
        lastAngle = angle


    }

    fun resetToAbsolute() {
        integratedAngleEncoder.position = (getEncoderMeasurement().degrees)
    }

    //TODO: use this method somewhere
    fun zeroEncoder():Double{
        encoder.reset()
        return encoder.positionOffset
    }

    fun getAngle(): Rotation2d {
        return Rotation2d.fromDegrees(integratedAngleEncoder.position)
    }

    fun getEncoderMeasurement(): Rotation2d {
        return Rotation2d.fromDegrees(encoder.absolutePosition)
    }

    fun getState(): SwerveModuleState {
        return SwerveModuleState(integratedDriveEncoder.getVelocity(), getAngle())
    }

    fun getPosition(): SwerveModulePosition {
        return SwerveModulePosition(integratedDriveEncoder.getPosition(), getAngle())
    }

}