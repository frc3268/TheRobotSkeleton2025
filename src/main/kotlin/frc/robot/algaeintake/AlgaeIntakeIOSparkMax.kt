package frc.robot.algaeintake

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
    }

    override fun updateInputs(inputs: AlgaeIntakeIO.Inputs) {
        inputs.jointVelocityMetersPerSec = jointMotor.getEncoder().velocity
        inputs.jointAppliedVolts = jointMotor.busVoltage
        inputs.jointCurrentAmps = doubleArrayOf(jointMotor.outputCurrent)
    }

    override fun setVoltage(voltage: Double) {
        jointMotor.setVoltage(voltage)
    }

    override fun toggle() {

    }

    override fun raiseFromBool(shouldRaise: Boolean) {
        if (shouldRaise) {
            raise()
        }
        else {
            lower()
        }
    }

    override fun raise() {
        //join.set(pidcontroller.calculuate(encoder measurmenet, 0.0)
        // or something idfk
    }
    override fun lower() {}

    override fun stopAll() {
        stopJoint()
        stopLeftAndRight()        
    }

    override fun stopJoint() {
        jointMotor.stopMotor()
    }

    override fun stopLeftAndRight() {
        leftMotor.stopMotor()
        rightMotor.stopMotor()
    }
}