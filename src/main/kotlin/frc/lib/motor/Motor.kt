package frc.lib.motor

import edu.wpi.first.math.controller.PIDController

interface Motor {

    /** ID of the motor. Should be unique for each motor */
    val ID: Int

    /** Should the motor be reversed? */
    var inverse: Boolean

    var positionPIDController: PIDController
    var velocityPIDController: PIDController

    /** Run the motor at the specified voltage */
    fun setVoltage(voltage: Double)
    /** Move the motor to the specified position */
    fun setPosition(position: Double)
    fun setVelocity(velocity: Double)

    fun getVelocityRPMMeasurement(): Double
    fun getAppliedVoltage(): Double
    fun getPositionDegreeMeasurement(): Double
    fun getCurrentAmps(): DoubleArray

    /** Configure the motor. It is best to refrain from calling this function */
    fun configure()

    /** Stop the motor **/
    fun stop()
    /** Close the motor and free up resources. This should not be called unless necessary **/
    fun close()
    /** Zero everything **/
    fun reset()

    /** Called periodically in simulation. Does nothing if motor is real */
    fun simulationPeriodic() {}
}