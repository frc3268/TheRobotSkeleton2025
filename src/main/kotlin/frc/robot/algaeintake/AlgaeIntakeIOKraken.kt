package frc.robot.algaeintake

import edu.wpi.first.math.controller.PIDController

class AlgaeIntakeIOKraken(override val pidController: PIDController) : AlgaeIntakeIO {
    override fun updateInputs(inputs: AlgaeIntakeIO.Inputs) {
        TODO("Not yet implemented")
    }

    override fun setJointVoltage(voltage: Double) {
        TODO("Not yet implemented")
    }

    override fun setMainAndRevVoltage(voltage: Double) {
        TODO("Not yet implemented")
    }

    override fun setRevVolate(voltage: Double) {
        TODO("Not yet implemented")
    }

    override fun setMainVoltage(voltage: Double) {
        TODO("Not yet implemented")
    }

    override fun stop() {
        TODO("Not yet implemented")
    }

    override fun stopMain() {
        TODO("Not yet implemented")
    }

    override fun stopRev() {
        TODO("Not yet implemented")
    }

    override fun stopJoint() {
        TODO("Not yet implemented")
    }
}