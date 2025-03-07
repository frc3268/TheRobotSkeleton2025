package frc.robot.elevator

import com.ctre.phoenix6.hardware.*
import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.*
import edu.wpi.first.units.measure.*
import frc.lib.swerve.ElevatorIO

class ElevatorIOKraken : ElevatorIO{
    val leftMotor = TalonFX(9, "rio")
    val rightMotor = TalonFX(10, "rio")

    override val pidController: PIDController = PIDController(0.125,0.005,0.0)

    init{
        val leftConfig = TalonFXConfiguration()
        val rightConfig = TalonFXConfiguration()

        leftConfig.Feedback.SensorToMechanismRatio = 0.0
        rightConfig.Feedback.SensorToMechanismRatio = 0.0

        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake

        leftConfig.CurrentLimits.StatorCurrentLimit = 120.0
        rightConfig.CurrentLimits.StatorCurrentLimit = 120.0

        leftConfig.CurrentLimits.StatorCurrentLimitEnable = true
        rightConfig.CurrentLimits.StatorCurrentLimitEnable = true

        leftMotor.configurator.apply(leftConfig)
        rightMotor.configurator.apply(rightConfig)
    }
    override fun updateInputs(inputs: ElevatorIO.Inputs) {
        //this formula may need to me changed to reflect the reality
        inputs.elevatorPositionMeters = (leftMotor.position.valueAsDouble + rightMotor.position.valueAsDouble) / 2
        inputs.rightMotorPositionMeters = rightMotor.position.valueAsDouble
        inputs.leftMotorPositionMeters = leftMotor.position.valueAsDouble
        inputs.rightMotorVelocityMetersPerSec = rightMotor.velocity.valueAsDouble
        inputs.leftMotorVelocityMetersPerSec = leftMotor.velocity.valueAsDouble
    }

    override fun setBothVolts(volts: Double) {
        rightMotor.setVoltage(volts)
        leftMotor.setVoltage(volts)
    }

    override fun reset() {
        //not totally sure if this works as intended
        //as intended means that it just changes the value reported by encoder
        rightMotor.setPosition(Angle.ofRelativeUnits(0.0, Units.Degree))
        leftMotor.setPosition(Angle.ofRelativeUnits(0.0, Units.Degree))
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