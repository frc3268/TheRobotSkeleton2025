package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.wpilibj2.command.SubsystemBase

class IntakeSubsystem:SubsystemBase() {

    val motor:CANSparkMax = CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless)

    fun stop() {
        motor.stopMotor()
    }

    fun set(){
        motor.set(0.3)
    }

    override fun periodic() {

    }

    override fun simulationPeriodic() {

    }
}