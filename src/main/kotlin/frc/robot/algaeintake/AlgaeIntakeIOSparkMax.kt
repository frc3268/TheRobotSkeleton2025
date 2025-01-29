package frc.robot.algaeintake

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController


class AlgaeIntakeIOSparkMax: AlgaeIntakeIO {
    val rightMotor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)
    var rightConfig = SparkMaxConfig()

    val leftMotor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)
    var leftConfig = SparkMaxConfig()

    val jointMotor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)
    var jointConfig = SparkMaxConfig()

    override val pidController = PIDController(0.0, 0.0, 0.0)

    init {
        jointConfig.encoder.positionConversionFactor(0.0)
        leftConfig.encoder.positionConversionFactor(0.0)
        rightConfig.encoder.positionConversionFactor(0.0)

        leftMotor.configure(leftConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        rightMotor.configure(rightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        jointMotor.configure(jointConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    override fun updateInputs(inputs: AlgaeIntakeIO.Inputs) {
        inputs.jointVelocityMetersPerSec = jointMotor.getEncoder().velocity
        inputs.jointAppliedVolts = jointMotor.busVoltage
        inputs.jointCurrentAmps = doubleArrayOf(jointMotor.outputCurrent)

        inputs.leftVelocityMetersPerSec = leftMotor.getEncoder().velocity
        inputs.leftAppliedVolts = leftMotor.busVoltage
        inputs.leftCurrentAmps = doubleArrayOf(leftMotor.outputCurrent)

        inputs.rightVelocityMetersPerSec = rightMotor.getEncoder().velocity
        inputs.rightAppliedVolts = rightMotor.busVoltage
        inputs.rightCurrentAmps = doubleArrayOf(rightMotor.outputCurrent)
    }

    override fun setJointVoltage(voltage: Double) {
        jointMotor.setVoltage(voltage)
    }

    override fun stop() {
        rightMotor.stopMotor()
        leftMotor.stopMotor()
        jointMotor.stopMotor()
    }

    override fun stopJoint() {
        jointMotor.stopMotor()
    }

    override fun stopLeft() {
        leftMotor.stopMotor()
    }

    override fun stopRight() {
        rightMotor.stopMotor()
    }


}