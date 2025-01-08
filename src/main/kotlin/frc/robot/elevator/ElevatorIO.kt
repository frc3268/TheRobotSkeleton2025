package frc.lib.swerve

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs


interface ElevatorIO {
    @AutoLog
    open class ElevatorIOInputs {
        var rightMotorPositionMeters: Double = 0.0
        var rightMotorVelocityMetersPerSec: Double = 0.0
        var rightMotorAppliedVolts: Double = 0.0
        var rightMotorCurrentAmps: DoubleArray = doubleArrayOf()

        var leftMotorPositionMeters: Double = 0.0
        var leftMotorVelocityMetersPerSec: Double = 0.0
        var leftMotorAppliedVolts: Double = 0.0
        var leftMotorCurrentAmps: DoubleArray = doubleArrayOf()

        var elevatorPositionMeters:Double = 0.0
    }

    val pidController:PIDController

    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: ElevatorIOInputs) {}

    /** Run the left motor at the specified voltage.  */
    fun setLeftVolts(volts: Double) {}

    /** Run the right motor at the specified voltage.  */
    fun setRightVolts(volts: Double) {}

    /** Run both motors at the specified voltage.  */
    fun setBothVolts(volts: Double) {}

    fun stop() {}

    /**Zero everything **/
    fun reset() {}
}

class ElevatorIOInputsAutoLogged : ElevatorIO.ElevatorIOInputs(), LoggableInputs {
    override fun toLog(table: LogTable) {
        table.put("rightMotorPositionMeters", rightMotorPositionMeters)
        table.put("leftMotorPositionMeters", leftMotorPositionMeters)
        table.put("rightMotorVelocityMetersPerSec", rightMotorVelocityMetersPerSec)
        table.put("leftMotorVelocityMetersPerSec", leftMotorVelocityMetersPerSec)
        table.put("rightMotorAppliedVolts", rightMotorAppliedVolts)
        table.put("leftMotorAppliedVolts", leftMotorAppliedVolts)
        table.put("rightMotorCurrentAmps", rightMotorCurrentAmps)
        table.put("leftMotorCurrentAmps", leftMotorCurrentAmps)
        table.put("elevatorPositionMeters", elevatorPositionMeters)
    }

    override fun fromLog(table: LogTable) {
        table.get("rightMotorPositionMeters", rightMotorPositionMeters)
        table.get("leftMotorPositionMeters", leftMotorPositionMeters)
        table.get("rightMotorVelocityMetersPerSec", rightMotorVelocityMetersPerSec)
        table.get("leftMotorVelocityMetersPerSec", leftMotorVelocityMetersPerSec)
        table.get("rightMotorAppliedVolts", rightMotorAppliedVolts)
        table.get("leftMotorAppliedVolts", leftMotorAppliedVolts)
        table.get("rightMotorCurrentAmps", rightMotorCurrentAmps)
        table.get("leftMotorCurrentAmps", leftMotorCurrentAmps)
        table.get("elevatorPositionMeters", elevatorPositionMeters)
    }
}
