package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.SubsystemBase

class ShooterSubsystem:SubsystemBase() {

    val pid:PIDController = PIDController(0.0, 0.0, 0.0, 0.0);
    val motor:CANSparkMax = CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless);

    public fun setShoot(velocityRotationsPerMinute:Double) {
        motor.set(pid.calculate(motor.encoder.velocity, velocityRotationsPerMinute))
    }
    public fun stop() {
        motor.stopMotor()
    }
    /** This method will be called once per scheduler run  */
    override fun periodic() {
    }

    /** This method will be called once per scheduler run during simulation  */
    override fun simulationPeriodic() {
    }
}