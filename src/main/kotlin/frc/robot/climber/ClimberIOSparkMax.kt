package frc.robot.climber

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController
import frc.lib.rotation2dFromRot
import frc.lib.swerve.ElevatorIO


class ClimberIOSparkMax : ClimberIO {
    val leftMotor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)

    var leftConfig = SparkMaxConfig()

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