package frc.robot.coralintake

import edu.wpi.first.math.controller.PIDController
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import edu.wpi.first.math.geometry.Rotation2d


interface CoralIntakeIO {
    @AutoLog
    open class Inputs {
        var jointAngle = Rotation2d()
        var jointVelocityRPM = 0.0

        var intakeVelocityRPM = 0.0
    }

    val pidController: PIDController

    class LoggedInputs : Inputs(), LoggableInputs {
        override fun toLog(table: LogTable) {
            table.put("jointAngle", jointAngle)
            table.put("jointVelocityRPM", jointVelocityRPM)

            table.put("intakeVelocityRPM", intakeVelocityRPM)
        }

        override fun fromLog(table: LogTable) {
            table.get("jointAngle", jointAngle)
            table.get("jointVelocityRPM", jointVelocityRPM)

            table.get("intakeVelocityRPM", intakeVelocityRPM)
        }
    }

    fun updateInputs(inputs: Inputs)

    fun setIntakeVoltage(voltage: Double)

    fun setJointVoltage(volatge: Double)

    fun stopJoint()

    fun stopIntake()

    fun stop()
}