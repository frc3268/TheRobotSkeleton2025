package frc.robot.climber

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController
import frc.lib.rotation2dFromRot
import frc.lib.swerve.ElevatorIO


class ClimberIOSparkMax : ClimberIO {
    val leftMotor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)
    val rightMotor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)

    var leftConfig = SparkMaxConfig()
    var rightConfig = SparkMaxConfig()

    override val pidController: PIDController = PIDController(0.0,0.0,0.0)

    init{
        leftConfig.encoder.positionConversionFactor(0.0)
        rightConfig.encoder.positionConversionFactor(0.0)

        leftMotor.configure(leftConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        rightMotor.configure(rightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }
    override fun updateInputs(inputs: ClimberIO.Inputs) {
        //this forumla may need to me changed to reflect the reality
        inputs.climberPositionDegrees = (leftMotor.encoder.position + rightMotor.encoder.position) / 2
        inputs.rightMotorVelocityDegreesPerSec = rightMotor.encoder.velocity * 6
        inputs.leftMotorVelocityDegreesPerSec = leftMotor.encoder.velocity * 6
        inputs.rightMotorAppliedVolts = rightMotor.busVoltage
        inputs.leftMotorAppliedVolts = leftMotor.busVoltage
        inputs.rightMotorCurrentAmps = doubleArrayOf(rightMotor.outputCurrent)
        inputs.leftMotorCurrentAmps = doubleArrayOf(leftMotor.outputCurrent)
        // I guess we're doing degrees now
        inputs.rightMotorPositionDegrees = rightMotor.encoder.position / 180
        inputs.leftMotorPositionDegrees = leftMotor.encoder.position / 180
    }

    override fun setBothVolts(volts: Double) {
        rightMotor.setVoltage(volts)
        leftMotor.setVoltage(volts)
    }

    override fun reset() {
        rightMotor.encoder.position = 0.0
        leftMotor.encoder.position = 0.0
    }

    override fun stop() {
        rightMotor.stopMotor()
        leftMotor.stopMotor()
    }

    override fun setLeftVolts(volts: Double) {
        leftMotor.setVoltage(volts)
    }

    override fun setRightVolts(volts: Double) {
        rightMotor.setVoltage(volts)
    }
}