package frc.robot.algaeintake

import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController
import frc.lib.rotation2dFromDeg


class AlgaeIntakeIOSparkMax: AlgaeIntakeIO {
    //right
    val mainMotor = SparkMax(15, SparkLowLevel.MotorType.kBrushless)
    var mainConfig = SparkMaxConfig()

    //left
    val revMotor = SparkMax(14, SparkLowLevel.MotorType.kBrushless)
    var revConfig = SparkMaxConfig()

    val jointMotor = SparkMax(17, SparkLowLevel.MotorType.kBrushless)
    var jointConfig = SparkMaxConfig()

    override val pidController = PIDController(0.07, 0.0, 0.0)


    init {
        jointConfig.encoder.positionConversionFactor(0.0)
        revConfig.encoder.positionConversionFactor(0.0)
        mainConfig.encoder.positionConversionFactor(0.0)
//
//        revMotor.configure(revConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
//        mainMotor.configure(mainConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
//        jointMotor.configure(jointConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    override fun updateInputs(inputs: AlgaeIntakeIO.Inputs) {
        inputs.jointAngle = (jointMotor.encoder.position * (360 / 27.0)).rotation2dFromDeg()
        inputs.jointVelocityMetersPerSec = jointMotor.getEncoder().velocity

        inputs.revVelocityMetersPerSec = revMotor.getEncoder().velocity

        inputs.mainVelocityMetersPerSec = mainMotor.getEncoder().velocity
    }

    override fun setJointVoltage(voltage: Double) {
        jointMotor.setVoltage(voltage)
    }
    override fun setMainAndRevVoltage(voltage: Double) {
        setRevVolate(voltage)
        setMainVoltage(voltage)
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

    override fun resetJointEncoder() {
        jointMotor.encoder.position = 0.0
    }
}