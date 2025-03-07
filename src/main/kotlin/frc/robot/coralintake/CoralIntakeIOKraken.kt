package frc.robot.coralintake

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import frc.lib.rotation2dFromDeg

class CoralIntakeIOKraken : CoralIntakeIO {
    val jointMotor = TalonFX(0, "rio")
    val intakeMotor = TalonFX(0, "rio")

    override val pidController: PIDController = PIDController(0.0,0.0,0.0)

    init{
        val jointConfig = TalonFXConfiguration()
        val intakeConfig = TalonFXConfiguration()

        jointConfig.Feedback.SensorToMechanismRatio = 0.0

        jointMotor.configurator.apply(jointConfig)
        intakeMotor.configurator.apply(intakeConfig)
    }

    // Not fully implemented
    override fun updateInputs(inputs: CoralIntakeIO.Inputs) {

        inputs.intakeVelocityRPM = intakeMotor.velocity.valueAsDouble
        inputs.jointVelocityRPM = jointMotor.velocity.valueAsDouble

        inputs.jointAngle = jointMotor.position.valueAsDouble.rotation2dFromDeg()

    }

    override fun setIntakeVoltage(voltage: Double) {
        intakeMotor.setVoltage(voltage)
    }

    override fun setJointVoltage(volatge: Double) {
        jointMotor.setVoltage(volatge)
    }

    override fun stopJoint() {
        jointMotor.stopMotor()
    }

    override fun stopIntake() {
        intakeMotor.stopMotor()
    }

    override fun reset() {
        TODO("Not yet implemented")
    }

    override fun stop() {
        stopIntake()
        stopJoint()
    }
}