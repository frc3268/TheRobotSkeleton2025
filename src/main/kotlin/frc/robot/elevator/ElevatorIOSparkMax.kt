package frc.robot.elevator

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController
import frc.lib.swerve.ElevatorIO


class ElevatorIOSparkMax(override val pidController: PIDController) : ElevatorIO {
    val leftMotor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)
    val rightMotor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)

    var leftConfig = SparkMaxConfig()
    var rightConfig = SparkMaxConfig()

    init{
        leftConfig.encoder.positionConversionFactor(0.0)
        rightConfig.encoder.positionConversionFactor(0.0)

        leftMotor.configure(leftConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        rightMotor.configure(rightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }
    override fun updateInputs(inputs: ElevatorIO.Inputs) {
        //this forumla may need to me changed to reflect the reality
        inputs.elevatorPositionMeters = (leftMotor.encoder.position + rightMotor.encoder.position) / 2
        inputs.rightMotorPositionMeters = rightMotor.encoder.position
        inputs.leftMotorPositionMeters = leftMotor.encoder.position
        inputs.rightMotorCurrentAmps = doubleArrayOf(rightMotor.outputCurrent)
        inputs.leftMotorCurrentAmps = doubleArrayOf(leftMotor.outputCurrent)
        inputs.rightMotorAppliedVolts = rightMotor.busVoltage * rightMotor.appliedOutput
        inputs.leftMotorAppliedVolts = leftMotor.busVoltage * leftMotor.appliedOutput
        inputs.rightMotorVelocityMetersPerSec = rightMotor.encoder.velocity
        inputs.leftMotorVelocityMetersPerSec = leftMotor.encoder.velocity
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