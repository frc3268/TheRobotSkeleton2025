package frc.robot.coralintake

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

    override val pidController: PIDController = PIDController(0.0,0.0,0.0)

    init {
        jointConfig.encoder.positionConversionFactor(0.0)
        intakeConfig.encoder.positionConversionFactor(0.0)

        intakeMotor.configure(intakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        jointMotor.configure(jointConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
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

    override fun stop() {
        stopIntake()
        stopJoint()
    }

    override fun updateInputs(inputs: CoralIntakeIO.Inputs) {
        inputs.intakeVelocityRPM = intakeMotor.getEncoder().velocity
        inputs.intakeAppliedVolts = intakeMotor.busVoltage
        inputs.intakeCurrentAmps = doubleArrayOf(intakeMotor.outputCurrent)
        inputs.jointVelocityRPM = jointMotor.getEncoder().velocity
        inputs.jointAppliedVolts = jointMotor.busVoltage
        inputs.jointCurrentAmps = doubleArrayOf(jointMotor.outputCurrent)
    }
}
