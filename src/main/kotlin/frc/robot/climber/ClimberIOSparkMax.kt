package frc.robot.climber

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.controller.PIDController


class ClimberIOSparkMax : ClimberIO {
    val leftMotor = SparkMax(16, SparkLowLevel.MotorType.kBrushless)

    override val pidController: PIDController = PIDController(0.0,0.0,0.0)

    init{

    }
    override fun updateInputs(inputs: ClimberIO.Inputs) {
        //this forumla may need to me changed to reflect the reality
        inputs.motorVelocityDegreesPerSec = leftMotor.encoder.velocity
        inputs.motorPositionDegrees = leftMotor.encoder.position
    }

    override fun setBothVolts(volts: Double) {
        leftMotor.setVoltage(volts)
    }

    override fun reset() {
        leftMotor.encoder.position = 0.0
    }

    override fun stop() {
        leftMotor.stopMotor()
    }

    override fun setLeftVolts(volts: Double) {
        leftMotor.setVoltage(volts)
    }

}