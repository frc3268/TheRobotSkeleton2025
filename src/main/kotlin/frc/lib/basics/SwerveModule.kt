package frc.lib.basics

import com.revrobotics.*
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.AnalogEncoder
import edu.wpi.first.wpilibj.shuffleboard.*
import frc.lib.constants.SwerveDriveConstants
import frc.lib.utils.rotation2dFromDeg
import kotlin.math.*

/*
Props: drive motor, drive encoder, angle motor, angle encoder, absolute encoder
Get: Cancoder measurement, Module state(velocity) and position
Set: Module state
 */
class SwerveModule(val moduleConstants: SwerveDriveConstants.ModuleConstants) {

    //shuffleboard
    private val ShuffleboardTab = Shuffleboard.getTab("Swerve Module" + (moduleConstants.MODULE_NUMBER))
    val setPointEntry:GenericEntry = ShuffleboardTab.add("Setpoint", 0.0).withWidget(BuiltInWidgets.kGyro).entry

    val angleEncoderEntry:GenericEntry = ShuffleboardTab.add("Angle Encoder(Relative)", 0.0).withWidget(BuiltInWidgets.kGyro).entry

    val absoluteEncoderEntry:GenericEntry = ShuffleboardTab.add("Angle Encoder(Absolute)", 0.0).withWidget(BuiltInWidgets.kGyro).entry


    private val driveMotor:CANSparkMax = CANSparkMax(moduleConstants.DRIVE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    private val angleMotor:CANSparkMax = CANSparkMax(moduleConstants.ANGLE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)

    private val driveEncoder:RelativeEncoder = driveMotor.encoder
    private val angleEncoder:RelativeEncoder = angleMotor.encoder

    private val absoluteEncoder:AnalogEncoder = AnalogEncoder(moduleConstants.ENCODER_ID)

    private var turnController:PIDController = moduleConstants.PID_CONTROLLER

    init {
        absoluteEncoder.distancePerRotation =
            SwerveDriveConstants.Encoder.POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION
        absoluteEncoder.positionOffset = moduleConstants.ANGLE_OFFSET.degrees
        driveEncoder.positionConversionFactor =
            SwerveDriveConstants.DriveMotor.POSITION_CONVERSION_FACTOR_METERS_PER_ROTATION
        driveEncoder.velocityConversionFactor =
            SwerveDriveConstants.DriveMotor.VELOCITY_CONVERSION_FACTOR_METERS_PER_SECOND
        angleEncoder.positionConversionFactor =
            SwerveDriveConstants.AngleMotor.POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION

        driveMotor.inverted = moduleConstants.DRIVE_MOTOR_REVERSED
        angleMotor.inverted = moduleConstants.ANGLE_MOTOR_REVERSED

        driveMotor.setOpenLoopRampRate(0.9)
        angleMotor.setOpenLoopRampRate(0.9)

        turnController.enableContinuousInput(
            -180.0, 180.0
        )

        driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 15)
        //todo: fix? below
        angleMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 15)


    }

    fun updateShuffleboard(){
        angleEncoderEntry.setDouble(getState().angle.degrees)
        absoluteEncoderEntry.setDouble(getAbsoluteEncoderMeasurement().degrees)
    }

    fun resetToAbsolute(){
        driveEncoder.position = 0.0
        angleEncoder.position = getAbsoluteEncoderMeasurement().degrees
    }
    private fun getAbsoluteEncoderMeasurement() : Rotation2d = ((absoluteEncoder.absolutePosition * 360.0) + moduleConstants.ANGLE_OFFSET.degrees).rotation2dFromDeg()
    fun getState() : SwerveModuleState = SwerveModuleState(driveEncoder.velocity, (angleEncoder.position.IEEErem(360.0).rotation2dFromDeg()))
    fun getPosition() : SwerveModulePosition = SwerveModulePosition(driveEncoder.position, angleEncoder.position.IEEErem(360.0).rotation2dFromDeg())

    fun setDesiredState(desiredState:SwerveModuleState){
        if (abs(desiredState.speedMetersPerSecond) < 0.01){
            stop()
            return
        }
        val optimizedState = SwerveModuleState.optimize(desiredState, getState().angle)
        setPointEntry.setDouble(optimizedState.angle.degrees)
        //TODO: 5.0 should be a const
        driveMotor.set(optimizedState.speedMetersPerSecond / 5.0)
        angleMotor.set(turnController.calculate(getState().angle.degrees, optimizedState.angle.degrees))
    }
    fun stop(){
        driveMotor.set(0.0)
        angleMotor.set(0.0)
    }



}