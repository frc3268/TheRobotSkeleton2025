package frc.robot.algaeintake

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface AlgaeIntakeIO {
    @AutoLog
    open class AlgaeIntakeIOInputs {
        var appliedVolts: Double = 0.0
        var velocityMetersPerSec: Double = 0.0
        var currentAmps: DoubleArray = doubleArrayOf()
    }

    fun updateInputs(inputs: AlgaeIntakeIOInputs)

    fun setVoltage(voltage: Double)
}