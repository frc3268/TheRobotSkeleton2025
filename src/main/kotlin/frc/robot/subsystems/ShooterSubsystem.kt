package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.networktables.BooleanEntry
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.SubsystemBase

class ShooterSubsystem:SubsystemBase() {

    val shootPid = PIDController(0.0, 0.0, 0.0, 0.0);
    val shootmotor = CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    val kP:Double = 0.0;
    val kI:Double = 0.0;
    val kD:Double = 0.0;
    val kMaxOutput:Double = 0.0;
    val kMinOutput:Double = 0.0;
    val maxRPM:Double = 0.0;
    var isSet:Boolean = false;

    fun setShoot(speed:Double) {
        shootmotor.set(speed)
    }

    fun toggleMotor(speed:Double) {
        if(isSet)
            stop()
        else
            setShoot(speed)
        isSet = !isSet
    }

    fun stop() {
        shootmotor.stopMotor()
        isSet = false
    }
    /** This method will be called once per scheduler run  */
    override fun periodic() {
    }

    /** This method will be called once per scheduler run during simulation  */
    override fun simulationPeriodic() {
    }
}