package frc.robot.algaeintake

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.controller.PIDController
import frc.lib.rotation2dFromDeg

class AlgaeIntakeIOKraken(override val pidController: PIDController) : AlgaeIntakeIO {

    val mainMotor = TalonFX(0, "rio")
    val revMotor = TalonFX(0, "rio")
    val jointMotor = TalonFX(0, "rio")

    init{
        val mainConfig = TalonFXConfiguration()
        val revConfig = TalonFXConfiguration()
        val jointConfig = TalonFXConfiguration()

        jointConfig.Feedback.SensorToMechanismRatio = 0.0

        mainMotor.configurator.apply(mainConfig)
        revMotor.configurator.apply(revConfig)
        jointMotor.configurator.apply(jointConfig)
    }

    override fun updateInputs(inputs: AlgaeIntakeIO.Inputs) {
        inputs.mainAppliedVolts = mainMotor.motorVoltage.valueAsDouble
        inputs.jointAppliedVolts = jointMotor.motorVoltage.valueAsDouble
        inputs.revAppliedVolts = revMotor.motorVoltage.valueAsDouble

        inputs.mainVelocityMetersPerSec = mainMotor.velocity.valueAsDouble
        inputs.jointVelocityMetersPerSec = jointMotor.velocity.valueAsDouble
        inputs.revVelocityMetersPerSec = revMotor.velocity.valueAsDouble

        inputs.mainCurrentAmps = doubleArrayOf(mainMotor.statorCurrent.valueAsDouble)
        inputs.jointCurrentAmps = doubleArrayOf(jointMotor.statorCurrent.valueAsDouble)
        inputs.revCurrentAmps = doubleArrayOf(revMotor.statorCurrent.valueAsDouble)

        inputs.jointAngle = jointMotor.position.valueAsDouble.rotation2dFromDeg()

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