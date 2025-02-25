package frc.lib.motor

import edu.wpi.first.math.controller.PIDController

interface Motor {

    val ID: Int
    var inverse: Boolean
    var positionPIDController: PIDController
    var velocityPIDController: PIDController

    /** Run the motor at the specified voltage.  */
    fun setVoltage(voltage: Double)
    fun setPosition(position: Double)
    fun setVelocity(velocity: Double)

    fun getVelocityRPMMeasurement(): Double
    fun getAppliedVoltage(): Double
    fun getPositionDegreeMeasurement(): Double
    fun getCurrentAmps(): DoubleArray

    /** Stop the motor **/
    fun stop()
    /** Close the motor and free up resources. This should not be called unless necessary **/
    fun close()
    /** Zero everything **/
    fun reset()
}