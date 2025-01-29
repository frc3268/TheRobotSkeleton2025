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

    val intakeMotor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)
    val intakeConfig = SparkMaxConfig()

    override val pidController = PIDController(0.0, 0.0, 0.0, 0.0)

    init {
        jointConfig.encoder.positionConversionFactor(0.0)
        jointMotor.configure(jointConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        intakeConfig.encoder.positionConversionFactor(0.0)
        intakeMotor.configure(intakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    override fun updateInputs(inputs: CoralIntakeIO.Inputs) {
        inputs.intakeVelocityRPM = intakeMotor.getEncoder().velocity
        inputs.intakeAppliedVolts = intakeMotor.busVoltage
        inputs.intakeCurrentAmps = doubleArrayOf(intakeMotor.outputCurrent)
        inputs.jointVelocityRPM = jointMotor.getEncoder().velocity
        inputs.jointAppliedVolts = jointMotor.busVoltage
        inputs.jointCurrentAmps = doubleArrayOf(jointMotor.outputCurrent)
    }

    override fun setWheelVoltage(voltage: Double) {
        intakeMotor.setVoltage(voltage)
    }

    override fun setAngle(angle: Rotation2d) {
        jointMotor.set(pidController.calculate(jointMotor.encoder.position, angle.degrees))
    }

    override fun stop() {
        intakeMotor.stopMotor()
    }
}
