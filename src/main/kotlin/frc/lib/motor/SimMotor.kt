package frc.lib.motor

import edu.wpi.first.math.controller.PIDController

class SimMotor(
    override val ID: Int,
    override var inverse: Boolean,
    override val positionPidController: PIDController,
    override val velocityPidController: PIDController,
) : Motor {
    override fun setVoltage(voltage: Double) {
        TODO("Not yet implemented")
    }

    override fun setPosition(position: Double) {
        TODO("Not yet implemented")
    }

    override fun setVelocity(velocity: Double) {
        TODO("Not yet implemented")
    }

    override fun getVelocityRPMMeasurement(): Double {
        TODO("Not yet implemented")
    }

    override fun getAppliedVoltage(): Double {
        TODO("Not yet implemented")
    }

    override fun getPositionDegreeMeasurement(): Double {
        TODO("Not yet implemented")
    }

    override fun getCurrentAmps(): DoubleArray {
        TODO("Not yet implemented")
    }

    override fun stop() {
        TODO("Not yet implemented")
    }

    override fun close() {
        TODO("Not yet implemented")
    }

    override fun reset() {
        TODO("Not yet implemented")
    }
}