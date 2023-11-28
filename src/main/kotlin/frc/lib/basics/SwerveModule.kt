package frc.lib.basics

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.AnalogEncoder
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import frc.lib.constants.SwerveDriveConstants
import frc.lib.utils.rotation2dFromDeg
import kotlin.math.abs

/*
Props: drive motor, drive encoder, angle motor, angle encoder, absolute encoder
Get: Cancoder measurement, Module state(velocity) and position
Set: Module state
 */
class SwerveModule(val moduleConstants: SwerveDriveConstants.ModuleConstants) {

    //shuffleboard
    private val ShuffleboardTab = Shuffleboard.getTab("Swerve Module" + (moduleConstants.MODULE_NUMBER))
    //todo: change to something else? Maybe?
    val setPointEntry:GenericEntry = ShuffleboardTab.add("Setpoint", 0.0).withWidget(BuiltInWidgets.kNumberBar).withProperties(mapOf("Min" to 0.0, "Max" to 360.0)).entry
    val angleEncoderEntry:GenericEntry = ShuffleboardTab.add("Angle Encoder", 0.0).withWidget(BuiltInWidgets.kNumberBar).withProperties(mapOf("Min" to 0.0, "Max" to 360.0)).entry
    
    private val driveMotor:CANSparkMax = CANSparkMax(moduleConstants.DRIVE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val angleMotor:CANSparkMax = CANSparkMax(moduleConstants.ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)

    private val driveEncoder:RelativeEncoder = driveMotor.encoder
    private val angleEncoder:RelativeEncoder = angleMotor.encoder

    private val absoluteEncoder:AnalogEncoder = AnalogEncoder(moduleConstants.ENCODER_ID)

    private var turnController:PIDController = moduleConstants.PID_CONTROLLER

    init {
        absoluteEncoder.distancePerRotation = SwerveDriveConstants.EncoderConsts.POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION
        absoluteEncoder.positionOffset = moduleConstants.ANGLE_OFFSET.degrees
        driveEncoder.positionConversionFactor = SwerveDriveConstants.DriveMotorConsts.POSITION_CONVERSION_FACTOR_METERS_PER_ROTATION
        driveEncoder.velocityConversionFactor = SwerveDriveConstants.DriveMotorConsts.VELOCITY_CONVERSION_FACTOR_METERS_PER_SECOND
        angleEncoder.positionConversionFactor = SwerveDriveConstants.AngleMotorConsts.POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION

        driveMotor.inverted = moduleConstants.DRIVE_MOTOR_REVERSED
        angleMotor.inverted = moduleConstants.ANGLE_MOTOR_REVERSED

        driveMotor.setOpenLoopRampRate(0.9)
        angleMotor.setOpenLoopRampRate(0.9)

        turnController.enableContinuousInput(
            -180.0,180.0
        )

        //todo: get this to work(https://github.com/orgs/frc3268/projects/2/views/1?pane=issue&itemId=43651204)
        //ShuffleboardTab.add("TurnController", turnController)
        ShuffleboardTab.add("Absolute Encoder", absoluteEncoder)
    }

    fun updateDashboard(){
        angleEncoderEntry.setDouble(getState().angle.degrees)
    }

    fun resetToAbsolute(){
        driveEncoder.position = 0.0
        angleEncoder.position = getAbsoluteEncoderMeasurement().degrees
    }
    private fun getAbsoluteEncoderMeasurement() : Rotation2d = ((absoluteEncoder.absolutePosition * 360.0) + moduleConstants.ANGLE_OFFSET.degrees).rotation2dFromDeg()
    fun getState() : SwerveModuleState = SwerveModuleState(driveEncoder.velocity, scopeAngle(angleEncoder.position.rotation2dFromDeg()))
    fun getPosition() : SwerveModulePosition = SwerveModulePosition(driveEncoder.position, scopeAngle(angleEncoder.position.rotation2dFromDeg()))

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

    fun scopeAngle(angle:Rotation2d) : Rotation2d{
        if(angle.degrees < 0){
            return scopeAngle(Rotation2d.fromDegrees(angle.degrees + 360.0))
        }
        if(angle.degrees > 360) {
            return scopeAngle(Rotation2d.fromDegrees(angle.degrees - 360.0))
        }
        return angle
    }


}