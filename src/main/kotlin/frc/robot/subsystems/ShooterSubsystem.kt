package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase

class ShooterSubsystem:SubsystemBase() {

    val pidController:PIDController = PIDController(0.0, 0.0, 0.0, 0.0);
    val leftFlywheelMotor:CANSparkMax = CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless)
    val rightFlywheelMotor:CANSparkMax = CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless)
    val indexWheelMotor:CANSparkMax = CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless)

    fun setShoot(speed:Double) {
        leftFlywheelMotor.set(-speed)
        rightFlywheelMotor.set(speed)
    }

    fun setIndex(){
       indexWheelMotor.set(0.3)
    }

    fun shootCommand(): Command {
        return run{
            setIndex()
        }.alongWith(run{
            setShoot(0.7)
        }).withTimeout(3.0).andThen(runOnce{stop()})
    }

    fun ampCommand(): Command{
        return run{
            setIndex()
        }.alongWith(run{
            setShoot(0.5)
        }).withTimeout(3.0).andThen(runOnce{stop()})
    }


    fun stop() {
        leftFlywheelMotor.stopMotor()
        indexWheelMotor.stopMotor()
    }
    /** This method will be called once per scheduler run  */
    override fun periodic() {
    }

    /** This method will be called once per scheduler run during simulation  */
    override fun simulationPeriodic() {
    }
}