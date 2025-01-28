package frc.robot.coralintake

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig




class CoralIntakeIOSparkMax : CoralIntakeIO {
    val motor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)
    var config = SparkMaxConfig()

    init {
        config.encoder.positionConversionFactor(0.0)
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    override fun updateInputs(inputs: CoralIntakeIO.Inputs) {
        inputs.velocityMetersPerSec = motor.getEncoder().velocity
        inputs.appliedVolts = motor.busVoltage
        inputs.currentAmps = doubleArrayOf(motor.outputCurrent)
    }

    override fun setWheelVoltage(voltage: Double) {
        motor.setVoltage(voltage)
    }

    override fun stop() {
        motor.stopMotor()
    }
}
