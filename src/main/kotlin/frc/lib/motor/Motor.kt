package frc.lib.motor

import edu.wpi.first.math.controller.PIDController

interface Motor {

    val positionPidController: PIDController
    val velocityPidController: PIDController
    var inverse: Boolean

    /** Run the motor at the specified voltage.  */
    fun setVoltage(voltage: Double)
    fun setPosition(position: Double)
    fun setVelocity(velocity: Double)

    fun getPositonMeasurement(): Double
    fun getVelocityRPMMeasurement(): Double
    fun getVelocityMetersPerSecMeasurement(): Double
    fun getAppliedVoltage(): Double
    fun getDegreeMeasurement(): Double

    /** Stop the motor **/
    fun stop()
    /** Close the motor and free up resources. This should not be called unless necessary **/
    fun close()
    /** Zero everything **/
    fun reset()
}