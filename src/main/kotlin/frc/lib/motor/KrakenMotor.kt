package frc.lib.motor

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.math.controller.PIDController

class KrakenMotor(
    override val ID: Int,
    override var inverse: Boolean = false,
    override val positionPidController: PIDController = PIDController(0.0,0.0,0.0),
    override val velocityPidController: PIDController = PIDController(0.0,0.0,0.0),
) : Motor {

    val motor = TalonFX(ID, "rio")

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

    override fun getVelocityRPMMeasurement(): Double {
        return motor.velocity.valueAsDouble
    }

    override fun getAppliedVoltage(): Double {
        return motor.motorVoltage.valueAsDouble
    }

    override fun getPositionDegreeMeasurement(): Double {
        return motor.position.valueAsDouble / 360
    }

    override fun getCurrentAmps(): DoubleArray {
        return doubleArrayOf(motor.statorCurrent.valueAsDouble)
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