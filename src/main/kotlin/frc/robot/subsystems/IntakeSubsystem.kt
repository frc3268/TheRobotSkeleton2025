package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.wpilibj2.command.SubsystemBase

class IntakeSubsystem:SubsystemBase() {

    val motor:CANSparkMax = CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless)

    public fun stop() {
        motor.stopMotor()
    }

    override fun periodic() {

    }

    override fun simulationPeriodic() {

    }
}