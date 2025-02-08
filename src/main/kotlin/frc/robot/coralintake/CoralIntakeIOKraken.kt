package frc.robot.coralintake

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController

class CoralIntakeIOKraken(override val pidController: PIDController) : CoralIntakeIO {
    override fun updateInputs(inputs: CoralIntakeIO.Inputs) {
        TODO("Not yet implemented")
    }

    override fun setIntakeVoltage(voltage: Double) {
        TODO("Not yet implemented")
    }

    override fun setJointVoltage(volatge: Double) {
        TODO("Not yet implemented")
    }

    override fun stopJoint() {
        TODO("Not yet implemented")
    }

    override fun stopIntake() {
        TODO("Not yet implemented")
    }

    override fun stop() {
        TODO("Not yet implemented")
    }

}