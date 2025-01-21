package frc.lib

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile

class Motor(id: Int, val positionPidController:ProfiledPIDController = ProfiledPIDController(0.0,0.0,0.0, TrapezoidProfile.Constraints(0.0,0.0)), val velocityPIDController: PIDController = PIDController(0.0,0.0,0.0), rotationsPerUnit: Double = 1.0) {
    val controller = SparkMax(id, SparkLowLevel.MotorType.kBrushless)
    var config: SparkMaxConfig = SparkMaxConfig()


    init{
        config.encoder.positionConversionFactor(1/rotationsPerUnit)
        controller.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    fun setPercentOutput(percentOutput: Double){
        controller.set(percentOutput / 100)
    }
    fun setVoltage(voltage: Double){
        controller.setVoltage(voltage)
    }
    fun setPosition(position: Double){
        positionPidController.goal = TrapezoidProfile.State(position, 0.0)
        controller.set(positionPidController.calculate(getPositonMeasurement()))
    }
    fun setVelocity(velocity: Double){
        velocityPIDController.setpoint = velocity
        controller.set(velocityPIDController.calculate(getVelocityMeasurement()))
    }
    fun stop(){
        controller.stopMotor()
    }

    fun isAtVelocityGoal() = velocityPIDController.atSetpoint()
    fun isAtPositionGoal() = positionPidController.atGoal()

    open fun getPositonMeasurement() = controller.getEncoder().position
    open fun getVelocityMeasurement() = controller.getEncoder().velocity
}
