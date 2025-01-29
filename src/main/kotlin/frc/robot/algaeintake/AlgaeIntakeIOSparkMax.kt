package frc.robot.algaeintake

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.coralintake.CoralIntakeIO


class AlgaeIntakeIOSparkMax : AlgaeIntakeIO {
    val jointMotor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)
    val jointConfig = SparkMaxConfig()

    val mainIntakeMotor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)
    val mainIntakeConfig = SparkMaxConfig()

    val revIntakeMotor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)
    val revIntakeConfig = SparkMaxConfig()

    override val pidController = PIDController(0.0, 0.0, 0.0, 0.0)

    init {
        jointConfig.encoder.positionConversionFactor(0.0)
        jointMotor.configure(jointConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        mainIntakeConfig.encoder.positionConversionFactor(0.0)
        mainIntakeMotor.configure(mainIntakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        revIntakeConfig.encoder.positionConversionFactor(0.0)
        revIntakeMotor.configure(revIntakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    override fun updateInputs(inputs: AlgaeIntakeIO.Inputs) {
        inputs.intakeVelocityRPM = (mainIntakeMotor.getEncoder().velocity + revIntakeMotor.getEncoder().velocity) / 2
        inputs.intakeAppliedVolts = (mainIntakeMotor.busVoltage + revIntakeMotor.busVoltage) / 2
        inputs.mainIntakeCurrentAmps = doubleArrayOf(mainIntakeMotor.outputCurrent)
        inputs.revIntakeCurrentAmps = doubleArrayOf(revIntakeMotor.outputCurrent)
        inputs.jointVelocityRPM = jointMotor.getEncoder().velocity
        inputs.jointAppliedVolts = jointMotor.busVoltage
        inputs.jointCurrentAmps = doubleArrayOf(jointMotor.outputCurrent)
    }

    override fun setWheelVoltage(voltage: Double) {
        mainIntakeMotor.setVoltage(voltage)
        revIntakeMotor.setVoltage(-voltage)
    }

    override fun setAngle(angle: Rotation2d) {
        jointMotor.set(pidController.calculate(jointMotor.encoder.position, angle.degrees))
    }

    override fun stop() {
        mainIntakeMotor.stopMotor()
        revIntakeMotor.stopMotor()
    }
}