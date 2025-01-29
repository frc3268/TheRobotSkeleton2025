package frc.robot.algaeintake

import frc.robot.coralintake.CoralIntakeIO
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d

interface AlgaeIntakeIO {
    @AutoLog
    open class Inputs {
        var jointAngle:Rotation2d = Rotation2d(0.0)
        var jointAppliedVolts: Double = 0.0
        var jointVelocityMetersPerSec: Double = 0.0
        var jointCurrentAmps: DoubleArray = doubleArrayOf()

        var rightAppliedVolts: Double = 0.0
        var rightVelocityMetersPerSec: Double = 0.0
        var rightCurrentAmps: DoubleArray = doubleArrayOf()

        var leftAppliedVolts: Double = 0.0
        var leftVelocityMetersPerSec: Double = 0.0
        var leftCurrentAmps: DoubleArray = doubleArrayOf()
    }

    val pidController: PIDController

    class LoggedInputs : Inputs(), LoggableInputs {
        override fun toLog(table: LogTable) {
            table.put("jointAngle", jointAngle)
            table.put("jointAppliedVolts", jointAppliedVolts)
            table.put("jointVelocityMetersPerSec", jointVelocityMetersPerSec)
            table.put("jointCurrentAmps", jointCurrentAmps)

            table.put("rightAppliedVolts", rightAppliedVolts)
            table.put("rightVelocityMetersPerSec", rightVelocityMetersPerSec)
            table.put("rightCurrentAmps", rightCurrentAmps)

            table.put("leftAppliedVolts", leftAppliedVolts)
            table.put("leftVelocityMetersPerSec", leftVelocityMetersPerSec)
            table.put("leftCurrentAmps", leftCurrentAmps)
        }

        override fun fromLog(table: LogTable) {
            table.get("jointAngle", jointAngle)
            table.get("jointAppliedVolts", jointAppliedVolts)
            table.get("jointVelocityMetersPerSec", jointVelocityMetersPerSec)
            table.get("jointCurrentAmps", jointCurrentAmps)

            table.get("rightAppliedVolts", rightAppliedVolts)
            table.get("rightVelocityMetersPerSec", rightVelocityMetersPerSec)
            table.get("rightCurrentAmps", rightCurrentAmps)

            table.get("leftAppliedVolts", leftAppliedVolts)
            table.get("leftVelocityMetersPerSec", leftVelocityMetersPerSec)
            table.get("leftCurrentAmps", leftCurrentAmps)
        }
    }

    fun updateInputs(inputs: Inputs)
    fun setJointVoltage(voltage: Double)
    fun setLeftAndRightVoltage(voltage: Double)
    fun setLeftVoltage(voltage: Double)
    fun setRightVoltage(voltage: Double)
    fun stop()
    fun stopRight()
    fun stopLeft()
    fun stopJoint()
}