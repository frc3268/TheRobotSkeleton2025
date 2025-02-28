package frc.lib.motor

import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.Slot1Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle


class KrakenMotor(
    override val id: Int,
    override var inverse: Boolean = false,
    val motorConfig: TalonFXConfiguration = TalonFXConfiguration(),
    var positionSlot: Slot0Configs = Slot0Configs(),
    var velocitySlot: Slot1Configs = Slot1Configs()
) : Motor {

    val motor = TalonFX(id, "rio")

    init{

        if (inverse) {
            motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        }
        else {
            motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive
        }


        motorConfig.Feedback.SensorToMechanismRatio = 0.0

        motor.configurator.apply(motorConfig)
        motor.configurator.apply(positionSlot)
        motor.configurator.apply(velocitySlot)
    }
    override fun configure() {
        motor.configurator.apply(motorConfig)
        motor.configurator.apply(positionSlot)
        motor.configurator.apply(velocitySlot)
    }

    override fun setVoltage(voltage: Double){
        motor.setVoltage(voltage)
    }

    override fun setPosition(position: Double) {
        motor.setPosition(position)
    }

    override fun setVelocity(velocity: Double) {
        val request = VelocityVoltage(velocity).withSlot(1);
        motor.setControl(request)
    }

    override fun getVelocityRPMMeasurement(): Double {
        return motor.velocity.valueAsDouble
    }

    override fun getAppliedVoltage(): Double {
        return motor.motorVoltage.valueAsDouble
    }

    override fun getPositionDegreeMeasurement(): Double {
        return getAppliedVoltage() / 360
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
        //not totally sure if this works as intended
        //as intended means that it just changes the value reported by encoder
        motor.setPosition(Angle.ofRelativeUnits(0.0, Units.Degree))
    }
}