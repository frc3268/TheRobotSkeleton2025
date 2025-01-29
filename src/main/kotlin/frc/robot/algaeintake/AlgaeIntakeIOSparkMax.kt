package frc.robot.algaeintake

import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkMaxConfig


class AlgaeIntakeIOSparkMax: AlgaeIntakeIO {
    val motor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)
    var config = SparkMaxConfig()

    init {
        config.encoder.positionConversionFactor(0.0)
    }

    override fun updateInputs(inputs: AlgaeIntakeIO.Inputs) {
        inputs.velocityMetersPerSec = motor.getEncoder().velocity
        inputs.appliedVolts = motor.busVoltage
        inputs.currentAmps = doubleArrayOf(motor.outputCurrent)
    }

    override fun setVoltage(voltage: Double) {
        motor.setVoltage(voltage)
    }

    override fun toggle() {

    }

    override fun riseFromBool(shouldRaise: Boolean) {
        if (shouldRaise) {
            raise()
        }
        else {
            lower()
        }
    }

    override fun raise() {}
    override fun lower() {}

    override fun stop() {
        motor.stopMotor()
    }
}