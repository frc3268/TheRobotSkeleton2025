package frc.robot.coralintake

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax

class CoralIntakeIOSparkMax : CoralIntakeIO {
    val motor = CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless)
    val encoder = motor.encoder

    init {
        encoder.positionConversionFactor = 0.0
    }

    override fun updateInputs(inputs: CoralIntakeIO.CoralIntakeIOInputs) {
        inputs.velocityMetersPerSec = encoder.velocity
        inputs.appliedVolts = motor.busVoltage
        inputs.currentAmps = doubleArrayOf(motor.outputCurrent)
    }

    override fun setVoltage(voltage: Double) {
        motor.setVoltage(voltage)
    }
}