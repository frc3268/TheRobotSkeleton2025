package frc.robot.algaeintake

import frc.robot.coralintake.CoralIntakeIO
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import edu.wpi.first.math.controller.PIDController

interface AlgaeIntakeIO {
    @AutoLog
    open class Inputs {
        var jointAppliedVolts: Double = 0.0
        var jointVelocityMetersPerSec: Double = 0.0
        var jointCurrentAmps: DoubleArray = doubleArrayOf()

        var rightAppliedVolts: Double = 0.0
        var rightVelocityMetersPerSec: Double = 0.0
        var rightCurrentAmps: DoubleArray = doubleArrayOf()
    }

    val pidController: PIDController

    class LoggedInputs : Inputs(), LoggableInputs {
        override fun toLog(table: LogTable) {
            table.put("jointAppliedVolts", jointAppliedVolts)
            table.put("jointVelocityMetersPerSec", jointVelocityMetersPerSec)
            table.put("jointCurrentAmps", jointCurrentAmps)
        }

        override fun fromLog(table: LogTable) {
            table.get("jointAppliedVolts", jointAppliedVolts)
            table.get("jointVelocityMetersPerSec", jointVelocityMetersPerSec)
            table.get("jointCurrentAmps", jointCurrentAmps)
        }
    }

    fun updateInputs(inputs: Inputs)

    fun setVoltage(voltage: Double)

    fun toggle()
    // Raise if shouldRaise is true
    fun raiseFromBool(shouldRaise: Boolean)
    fun raise()
    fun lower()

    fun stopAll()
}