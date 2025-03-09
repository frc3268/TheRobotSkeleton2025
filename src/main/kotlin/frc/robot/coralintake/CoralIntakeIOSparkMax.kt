package frc.robot.coralintake

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.controller.PIDController
import frc.lib.rotation2dFromDeg

class CoralIntakeIOSparkMax : CoralIntakeIO {
    val jointMotor = SparkMax(15, SparkLowLevel.MotorType.kBrushless)

    val intakeMotor = SparkMax(14, SparkLowLevel.MotorType.kBrushless)

    override val pidController: PIDController = PIDController(0.04,0.005,0.0)

    init {
        //jointConfig.encoder.positionConversionFactor(0.01)
        //intakeConfig.encoder.positionConversionFactor(0.0)

        //intakeMotor.configure(intakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        //jointMotor.configure(jointConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)


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
        inputs.jointVelocityRPM = jointMotor.getEncoder().velocity * (360 / 28)
        inputs.jointAngle = (jointMotor.encoder.position * (360 / 28) ).rotation2dFromDeg()
    }

    override fun reset() {
        jointMotor.encoder.position = 0.0
    }


}
