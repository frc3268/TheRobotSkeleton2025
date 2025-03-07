package frc.robot.climber

import edu.wpi.first.math.controller.PIDController
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs


interface ClimberIO {
    @AutoLog
    open class Inputs {
        var motorPositionDegrees = 0.0
        var motorVelocityDegreesPerSec = 0.0

    }

    class LoggedInputs : Inputs(), LoggableInputs {
        override fun toLog(table: LogTable) {
            table.put("leftMotorPositionMeters", motorPositionDegrees)
            table.put("leftMotorVelocityMetersPerSec", motorVelocityDegreesPerSec)
        }

        override fun fromLog(table: LogTable) {
            table.get("leftMotorPositionMeters", motorPositionDegrees)
            table.get("leftMotorVelocityMetersPerSec", motorVelocityDegreesPerSec)
        }
    }


    val pidController: PIDController

    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: Inputs)

    /** Run the left motor at the specified voltage.  */
    fun setLeftVolts(volts: Double)

    /** Run both motors at the specified voltage.  */
    fun setBothVolts(volts: Double)

    fun stop()

    /**Zero everything **/
    fun reset()
}
