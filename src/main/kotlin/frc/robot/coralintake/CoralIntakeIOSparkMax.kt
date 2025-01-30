package frc.robot.coralintake

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController


class CoralIntakeIOSparkMax : CoralIntakeIO {
    val jointMotor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)
    var config = SparkMaxConfig()

    val wheelMotor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)

    override val pidController: PIDController = PIDController(0.0,0.0,0.0)

    init {
        config.encoder.positionConversionFactor(0.0)
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
}
