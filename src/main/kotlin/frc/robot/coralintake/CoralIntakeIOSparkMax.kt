package frc.robot.coralintake

import edu.wpi.first.math.geometry.Rotation2d
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController

class CoralIntakeIOSparkMax : CoralIntakeIO {
    val jointMotor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)
    val jointConfig = SparkMaxConfig()

    val wheelMotor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)
    val wheelConfig = SparkMaxConfig()

    override val pidController: PIDController = PIDController(0.0,0.0,0.0)

    init {
        jointConfig.encoder.positionConversionFactor(0.0)
        jointMotor.configure(jointConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        wheelConfig.encoder.positionConversionFactor(0.0)
        wheelMotor.configure(intakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        jointMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    override fun updateInputs(inputs: CoralIntakeIO.Inputs) {
        inputs.velocityMetersPerSec = jointMotor.getEncoder().velocity
        inputs.appliedVolts = jointMotor.busVoltage
        inputs.currentAmps = doubleArrayOf(jointMotor.outputCurrent)
    }

    override fun setWheelVoltage(voltage: Double) {
        wheelMotor.setVoltage(voltage)
    }

    override fun setJointVoltage(volatge: Double) {
        jointMotor.setVoltage(volatge)
    }

    override fun stopJoint() {
        jointMotor.stopMotor()
    }

    override fun stopWheel() {
         wheelMotor.stopMotor()
    }

    override fun stop() {
        stopWheel()
        stopJoint()
    }

    override fun updateInputs(inputs: CoralIntakeIO.Inputs) {
        inputs.intakeVelocityRPM = wheelMotor.getEncoder().velocity
        inputs.intakeAppliedVolts = WheelMotor.busVoltage
        inputs.intakeCurrentAmps = doubleArrayOf(wheelMotor.outputCurrent)
        inputs.jointVelocityRPM = jointMotor.getEncoder().velocity
        inputs.jointAppliedVolts = jointMotor.busVoltage
        inputs.jointCurrentAmps = doubleArrayOf(jointMotor.outputCurrent)
    }
    }
}
