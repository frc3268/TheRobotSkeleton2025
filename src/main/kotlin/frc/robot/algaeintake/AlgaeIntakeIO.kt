package frc.robot.algaeintake

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface AlgaeIntakeIO {
    @AutoLog
    open class Inputs {
        var jointAngle:Rotation2d = Rotation2d(0.0)
        var jointAppliedVolts: Double = 0.0
        var jointVelocityMetersPerSec: Double = 0.0
        var jointCurrentAmps: DoubleArray = doubleArrayOf()

        var mainAppliedVolts: Double = 0.0
        var mainVelocityMetersPerSec: Double = 0.0
        var mainCurrentAmps: DoubleArray = doubleArrayOf()

        var revAppliedVolts: Double = 0.0
        var revVelocityMetersPerSec: Double = 0.0
        var revCurrentAmps: DoubleArray = doubleArrayOf()
    }

    val pidController: PIDController

    class LoggedInputs : Inputs(), LoggableInputs {
        override fun toLog(table: LogTable) {
            table.put("jointAngle", jointAngle)
            table.put("jointAppliedVolts", jointAppliedVolts)
            table.put("jointVelocityMetersPerSec", jointVelocityMetersPerSec)
            table.put("jointCurrentAmps", jointCurrentAmps)

            table.put("rightAppliedVolts", mainAppliedVolts)
            table.put("rightVelocityMetersPerSec", mainVelocityMetersPerSec)
            table.put("rightCurrentAmps", mainCurrentAmps)

            table.put("leftAppliedVolts", revAppliedVolts)
            table.put("leftVelocityMetersPerSec", revVelocityMetersPerSec)
            table.put("leftCurrentAmps", revCurrentAmps)
        }

        override fun fromLog(table: LogTable) {
            table.get("jointAngle", jointAngle)
            table.get("jointAppliedVolts", jointAppliedVolts)
            table.get("jointVelocityMetersPerSec", jointVelocityMetersPerSec)
            table.get("jointCurrentAmps", jointCurrentAmps)

            table.get("rightAppliedVolts", mainAppliedVolts)
            table.get("rightVelocityMetersPerSec", mainVelocityMetersPerSec)
            table.get("rightCurrentAmps", mainCurrentAmps)

            table.get("leftAppliedVolts", revAppliedVolts)
            table.get("leftVelocityMetersPerSec", revVelocityMetersPerSec)
            table.get("leftCurrentAmps", revCurrentAmps)
        }
    }

    fun updateInputs(inputs: Inputs)
    fun setJointVoltage(voltage: Double)
    fun setMainAndRevVoltage(voltage: Double)
    fun setRevVolate(voltage: Double)
    fun setMainVoltage(voltage: Double)
    fun stop()
    fun stopMain()
    fun stopRev()
    fun stopJoint()
}