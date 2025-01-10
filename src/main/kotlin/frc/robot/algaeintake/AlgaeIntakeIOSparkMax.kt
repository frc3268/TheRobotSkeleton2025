package frc.robot.algaeintake

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import frc.robot.coralintake.CoralIntakeIO

class AlgaeIntakeIOSparkMax: AlgaeIntakeIO {
    val motor = CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless)
    val encoder = motor.encoder

    init {
        encoder.positionConversionFactor = 0.0
    }

    override fun updateInputs(inputs: AlgaeIntakeIO.Inputs) {
        inputs.velocityMetersPerSec = encoder.velocity
        inputs.appliedVolts = motor.busVoltage
        inputs.currentAmps = doubleArrayOf(motor.outputCurrent)
    }

    override fun setVoltage(voltage: Double) {
        motor.setVoltage(voltage)
    }

    override fun stop() {
        motor.stopMotor()
    }
}