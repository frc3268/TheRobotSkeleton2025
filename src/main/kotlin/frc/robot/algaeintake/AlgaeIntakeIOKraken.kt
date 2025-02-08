package frc.robot.algaeintake

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.controller.PIDController

class AlgaeIntakeIOKraken(override val pidController: PIDController) : AlgaeIntakeIO {

    val mainMotor = TalonFX(0, "rio")
    val revMotor = TalonFX(0, "rio")
    val jointMotor = TalonFX(0, "rio")

    init{
        val mainConfig = TalonFXConfiguration()
        val revConfig = TalonFXConfiguration()
        val jointConfig = TalonFXConfiguration()

        mainMotor.configurator.apply(mainConfig)
        revMotor.configurator.apply(revConfig)
        jointMotor.configurator.apply(jointConfig)
    }

    override fun updateInputs(inputs: AlgaeIntakeIO.Inputs) {
        TODO("Not yet implemented")
    }

    override fun setJointVoltage(voltage: Double) {
        jointMotor.setVoltage(voltage)
    }

    override fun setMainAndRevVoltage(voltage: Double) {
        setRevVolate(voltage)
        setMainVoltage(-voltage)
    }

    override fun setRevVolate(voltage: Double) {
        revMotor.setVoltage(voltage)
    }

    override fun setMainVoltage(voltage: Double) {
        mainMotor.setVoltage(voltage)
    }

    override fun stop() {
        stopJoint()
        stopRev()
        stopMain()
    }

    override fun stopMain() {
        mainMotor.stopMotor()
    }

    override fun stopRev() {
        revMotor.stopMotor()
    }

    override fun stopJoint() {
        jointMotor.stopMotor()
    }
}