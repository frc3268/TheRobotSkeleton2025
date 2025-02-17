package frc.robot.climber

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle

class ClimberIOKraken : ClimberIO{
    val leftMotor = TalonFX(0, "rio")
    val rightMotor = TalonFX(0, "rio")

    override val pidController: PIDController = PIDController(0.0,0.0,0.0)

    init{
        val leftConfig = TalonFXConfiguration()
        val rightConfig = TalonFXConfiguration()

        leftConfig.Feedback.SensorToMechanismRatio = 0.0
        rightConfig.Feedback.SensorToMechanismRatio = 0.0

        leftMotor.configurator.apply(leftConfig)
        rightMotor.configurator.apply(rightConfig)
    }
    override fun updateInputs(inputs: ClimberIO.Inputs) {
        // TODO: THIS
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