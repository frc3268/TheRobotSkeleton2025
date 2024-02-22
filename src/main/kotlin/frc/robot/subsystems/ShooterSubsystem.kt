package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase

class ShooterSubsystem:SubsystemBase() {

    val leftFlywheelMotor:CANSparkMax = CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless)
    val rightFlywheelMotor:CANSparkMax = CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless)
    fun setShoot(speed:Double) {
        leftFlywheelMotor.set(-speed)
        rightFlywheelMotor.set(speed)
    }


    fun shootCommand(): Command {
        return run{
            setShoot(0.7)
        }.withTimeout(3.0).andThen(runOnce{stop()})
    }

    fun ampCommand(): Command{
        return run{
            setShoot(0.5)
        }.withTimeout(3.0).andThen(runOnce{stop()})
    }

    fun takeInCommand(): Command {
        return run{
            setShoot(-0.7)
        }.withTimeout(3.0)
    }


    fun stop() {
        leftFlywheelMotor.stopMotor()
        rightFlywheelMotor.stopMotor()
    }
    /** This method will be called once per scheduler run  */
    override fun periodic() {
    }

    /** This method will be called once per scheduler run during simulation  */
    override fun simulationPeriodic() {
    }
}