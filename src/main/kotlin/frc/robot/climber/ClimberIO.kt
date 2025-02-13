package frc.robot.climber

import edu.wpi.first.math.controller.PIDController
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs


interface ClimberIO {
    @AutoLog
    open class Inputs {
        var rightMotorPositionDegrees = 0.0
        var rightMotorVelocityDegreesPerSec = 0.0
        var rightMotorAppliedVolts = 0.0
        var rightMotorCurrentAmps = doubleArrayOf()

        var leftMotorPositionDegrees = 0.0
        var leftMotorVelocityDegreesPerSec = 0.0
        var leftMotorAppliedVolts = 0.0
        var leftMotorCurrentAmps = doubleArrayOf()

        var climberPositionDegrees = 0.0
    }

    class LoggedInputs : Inputs(), LoggableInputs {
        override fun toLog(table: LogTable) {
            table.put("rightMotorPositionMeters", rightMotorPositionDegrees)
            table.put("leftMotorPositionMeters", leftMotorPositionDegrees)
            table.put("rightMotorVelocityMetersPerSec", rightMotorVelocityDegreesPerSec)
            table.put("leftMotorVelocityMetersPerSec", leftMotorVelocityDegreesPerSec)
            table.put("rightMotorAppliedVolts", rightMotorAppliedVolts)
            table.put("leftMotorAppliedVolts", leftMotorAppliedVolts)
            table.put("rightMotorCurrentAmps", rightMotorCurrentAmps)
            table.put("leftMotorCurrentAmps", leftMotorCurrentAmps)
            table.put("elevatorPositionMeters", climberPositionDegrees)
        }

        override fun fromLog(table: LogTable) {
            table.get("rightMotorPositionMeters", rightMotorPositionDegrees)
            table.get("leftMotorPositionMeters", leftMotorPositionDegrees)
            table.get("rightMotorVelocityMetersPerSec", rightMotorVelocityDegreesPerSec)
            table.get("leftMotorVelocityMetersPerSec", leftMotorVelocityDegreesPerSec)
            table.get("rightMotorAppliedVolts", rightMotorAppliedVolts)
            table.get("leftMotorAppliedVolts", leftMotorAppliedVolts)
            table.get("rightMotorCurrentAmps", rightMotorCurrentAmps)
            table.get("leftMotorCurrentAmps", leftMotorCurrentAmps)
            table.get("elevatorPositionMeters", climberPositionDegrees)
        }
    }


    val pidController:PIDController

    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: Inputs)

    /** Run the left motor at the specified voltage.  */
    fun setLeftVolts(volts: Double)

    /** Run the right motor at the specified voltage.  */
    fun setRightVolts(volts: Double)

    /** Run both motors at the specified voltage.  */
    fun setBothVolts(volts: Double)

    fun stop()

    /**Zero everything **/
    fun reset()
}
