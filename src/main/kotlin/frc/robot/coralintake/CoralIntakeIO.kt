package frc.robot.coralintake

import org.littletonrobotics.junction.AutoLog

interface CoralIntakeIO {
    @AutoLog
    open class CoralIntakeIOInputs {
        var appliedVolts: Double = 0.0
        var velocityMetersPerSec: Double = 0.0
        var currentAmps: DoubleArray = doubleArrayOf()
    }

    fun updateInputs(inputs: CoralIntakeIOInputs)

    fun setVoltage(voltage: Double)
}