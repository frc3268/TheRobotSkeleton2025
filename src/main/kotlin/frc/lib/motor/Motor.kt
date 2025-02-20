package frc.lib.motor

import edu.wpi.first.math.controller.PIDController
import frc.robot.climber.ClimberIO

interface Motor {


    val pidController: PIDController
    var inverse: Boolean

    /** Run the motor at the specified voltage.  */
    fun setVoltage(voltage: Double)
    fun setPosition(position: Double)
    fun setVelocity(velocity: Double)

    fun getPositonMeasurement()
    fun getVelocityRPMMeasurement()
    fun getVelocityMetersPerSecMeasurement(): Double
    fun getAppliedVoltage()
    fun getDegreeMeasurement()


    /** Stop the motor **/
    fun stop()

    /** Close the motor and free up resources. This should not be called unless necessary **/
    fun close()

    /** Zero everything **/
    fun reset()
}