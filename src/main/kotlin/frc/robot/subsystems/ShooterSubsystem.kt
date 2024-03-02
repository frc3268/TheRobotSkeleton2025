package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

class ShooterSubsystem: SubsystemBase() {
    val leftFlywheelMotor = CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless)
    val rightFlywheelMotor = CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless)

    fun shootAtSpeed(speed: Double) {
        leftFlywheelMotor.set(speed)
        rightFlywheelMotor.set(speed)
    }

    fun shootAtSpeedCommand(speed: Double): Command =
        runOnce { shootAtSpeed(speed) }

    fun shootCommand(): Command =
        runOnce { shootAtSpeed(-1.0) }

    fun ampCommand(): Command =
        shootAtSpeedCommand(-0.5)

    fun takeInCommand(): Command =
        run { shootAtSpeed(0.7) }
            // TODO Adjust timeout
            .withTimeout(0.5)
            .andThen(stopCommand())

    fun stop() {
        leftFlywheelMotor.stopMotor()
        rightFlywheelMotor.stopMotor()
    }

    fun stopCommand(): Command =
        runOnce { stop() }

    /** This method will be called once per scheduler run  */
    override fun periodic() {
    }

    /** This method will be called once per scheduler run during simulation  */
    override fun simulationPeriodic() {
    }
}