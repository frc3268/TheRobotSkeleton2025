package frc.robot.algaeintake

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController
import frc.lib.rotation2dFromDeg


class AlgaeIntakeIOSparkMax: AlgaeIntakeIO {
    val mainMotor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)
    var mainConfig = SparkMaxConfig()

    val revMotor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)
    var revConfig = SparkMaxConfig()

    val jointMotor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)
    var jointConfig = SparkMaxConfig()

    override val pidController = PIDController(0.0, 0.0, 0.0)

    init {
        jointConfig.encoder.positionConversionFactor(0.0)
        revConfig.encoder.positionConversionFactor(0.0)
        mainConfig.encoder.positionConversionFactor(0.0)

        revMotor.configure(revConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        mainMotor.configure(mainConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        jointMotor.configure(jointConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    override fun updateInputs(inputs: AlgaeIntakeIO.Inputs) {
        inputs.jointAngle = jointMotor.encoder.position.rotation2dFromDeg()
        inputs.jointVelocityMetersPerSec = jointMotor.getEncoder().velocity
        inputs.jointAppliedVolts = jointMotor.busVoltage
        inputs.jointCurrentAmps = doubleArrayOf(jointMotor.outputCurrent)

        inputs.revVelocityMetersPerSec = revMotor.getEncoder().velocity
        inputs.revAppliedVolts = revMotor.busVoltage
        inputs.revCurrentAmps = doubleArrayOf(revMotor.outputCurrent)

        inputs.mainVelocityMetersPerSec = mainMotor.getEncoder().velocity
        inputs.mainAppliedVolts = mainMotor.busVoltage
        inputs.mainCurrentAmps = doubleArrayOf(mainMotor.outputCurrent)
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

    override fun stopJoint() {
        jointMotor.stopMotor()
    }

    override fun stopRev() {
        revMotor.stopMotor()
    }

    override fun stopMain() {
        mainMotor.stopMotor()
    }
}