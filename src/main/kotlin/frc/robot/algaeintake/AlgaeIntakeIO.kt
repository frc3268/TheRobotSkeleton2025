package frc.robot.algaeintake

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.coralintake.CoralIntakeIO
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface AlgaeIntakeIO {
    @AutoLog
    open class Inputs {
        var jointAngle = Rotation2d()
        var jointVelocityRPM = 0.0
        var jointAppliedVolts = 0.0
        var jointCurrentAmps = doubleArrayOf()

        var intakeVelocityRPM = 0.0
        var intakeAppliedVolts = 0.0
        var mainIntakeCurrentAmps = doubleArrayOf()
        var revIntakeCurrentAmps = doubleArrayOf()
    }

    val pidController: PIDController

    class LoggedInputs : Inputs(), LoggableInputs {
        override fun toLog(table: LogTable) {
            table.put("jointAngle", jointAngle)
            table.put("jointVelocityRPM", jointVelocityRPM)
            table.put("jointAppliedVolts", jointAppliedVolts)
            table.put("jointCurrentAmps", jointCurrentAmps)

            table.put("intakeVelocityRPM", intakeVelocityRPM)
            table.put("intakeAppliedVolts", intakeAppliedVolts)
            table.put("mainIntakeCurrentAmps", mainIntakeCurrentAmps)
            table.put("revIntakeCurrentAmps", revIntakeCurrentAmps)
        }

        override fun fromLog(table: LogTable) {
            table.get("jointAngle", jointAngle)
            table.get("jointVelocityRPM", jointVelocityRPM)
            table.get("jointAppliedVolts", jointAppliedVolts)
            table.get("jointCurrentAmps", jointCurrentAmps)

            table.get("intakeVelocityRPM", intakeVelocityRPM)
            table.get("intakeAppliedVolts", intakeAppliedVolts)
            table.get("revIntakeCurrentAmps", revIntakeCurrentAmps)
        }
    }

    fun updateInputs(inputs: Inputs)

    fun setWheelVoltage(voltage: Double)

    fun setAngle(angle: Rotation2d)

    fun stop()
}