package frc.lib.motor

import com.revrobotics.spark.ClosedLoopSlot
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkBase.ControlType
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController

// We should use this class more fr fr
// Also not use as much PiD Controllers
class SparkMaxMotor(
    override val ID: Int,
    override var inverse: Boolean = false,
    override var positionPIDController: PIDController,
    override var velocityPIDController: PIDController,
) : Motor {

    val motor = SparkMax(ID, SparkLowLevel.MotorType.kBrushless)
    var motorClosedLoop = motor.closedLoopController;
    var motorConfig = SparkMaxConfig()

    init{
        motorConfig.inverted(inverse)
        motorConfig.closedLoop
            .p(positionPIDController.p)
            .i(positionPIDController.i)
            .d(positionPIDController.d)
            .p(velocityPIDController.p, ClosedLoopSlot.kSlot1)
            .i(velocityPIDController.i, ClosedLoopSlot.kSlot1)
            .d(velocityPIDController.d, ClosedLoopSlot.kSlot1)

        motor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters)
    }

    override fun setVoltage(voltage: Double) {
        motor.setVoltage(voltage)
    }

    override fun setPosition(position: Double) {
        motorClosedLoop.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0)
    }

    override fun setVelocity(velocity: Double) {
        motorClosedLoop.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1)
    }


    override fun getVelocityRPMMeasurement(): Double {
        return motor.getEncoder().velocity
    }

    override fun getAppliedVoltage(): Double {
        return motor.busVoltage
    }

    override fun getPositionDegreeMeasurement(): Double {
        return getAppliedVoltage() / 360
    }

    override fun getCurrentAmps(): DoubleArray {
        return doubleArrayOf(motor.outputCurrent)
    }

    override fun stop() {
        motor.stopMotor()
    }

    override fun close() {
        motor.close()
    }

    override fun reset() {
        motor.encoder.position = 0.0
    }
}
