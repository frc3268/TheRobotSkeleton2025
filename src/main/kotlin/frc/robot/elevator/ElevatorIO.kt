package frc.lib.swerve

import edu.wpi.first.math.controller.PIDController
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs


interface ElevatorIO {
    @AutoLog
    open class Inputs {
        var rightMotorPositionMeters = 0.0
        var rightMotorVelocityMetersPerSec = 0.0

        var leftMotorPositionMeters = 0.0
        var leftMotorVelocityMetersPerSec = 0.0

        var elevatorPositionMeters = 0.0
    }

    class LoggedInputs : Inputs(), LoggableInputs {
        override fun toLog(table: LogTable) {
            table.put("rightMotorPositionMeters", rightMotorPositionMeters)
            table.put("leftMotorPositionMeters", leftMotorPositionMeters)
            table.put("rightMotorVelocityMetersPerSec", rightMotorVelocityMetersPerSec)
            table.put("leftMotorVelocityMetersPerSec", leftMotorVelocityMetersPerSec)
            table.put("elevatorPositionMeters", elevatorPositionMeters)
        }

        override fun fromLog(table: LogTable) {
            table.get("rightMotorPositionMeters", rightMotorPositionMeters)
            table.get("leftMotorPositionMeters", leftMotorPositionMeters)
            table.get("rightMotorVelocityMetersPerSec", rightMotorVelocityMetersPerSec)
            table.get("leftMotorVelocityMetersPerSec", leftMotorVelocityMetersPerSec)
            table.get("elevatorPositionMeters", elevatorPositionMeters)
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
