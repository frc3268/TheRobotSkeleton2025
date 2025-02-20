package frc.lib.motor

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.math.controller.PIDController

class KrakenMotor(id: Int,
                  override val pidController: PIDController = PIDController(0.0,0.0,0.0),
                  override var inverse: Boolean
) : Motor {
    val motor = TalonFX(id, "rio")

    init{
        val motorConfig = TalonFXConfiguration()

        if (inverse) {
            motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        }
        else {
            motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive
        }

        motorConfig.Feedback.SensorToMechanismRatio = 0.0

        motor.configurator.apply(motorConfig)
    }


    override fun setVoltage(voltage: Double){
        motor.setVoltage(voltage)
    }

    override fun setPosition(position: Double) {
        TODO("Not yet implemented")
    }

    override fun setVelocity(velocity: Double) {
        TODO("Not yet implemented")
    }

    override fun getPositonMeasurement() {
        TODO("Not yet implemented")
    }

    override fun getVelocityRPMMeasurement() {
        TODO("Not yet implemented")
    }

    override fun getVelocityMetersPerSecMeasurement(): Double {
        return motor.velocity.valueAsDouble
    }

    override fun getAppliedVoltage() {
        TODO("Not yet implemented")
    }

    override fun getDegreeMeasurement() {
        TODO("Not yet implemented")
    }

    override fun stop() {
        motor.stopMotor()
    }

    override fun close() {
        motor.close()
    }

    override fun reset() {
        TODO("Not yet implemented")
    }
}