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

    val shootPid:PIDController = PIDController(0.0, 0.0, 0.0, 0.0);
    val shootmotor:CANSparkMax = CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    public val kP:Double = 0.0;
    public val kI:Double = 0.0;
    public val kD:Double = 0.0;
    public val kMaxOutput:Double = 0.0;
    public val kMinOutput:Double = 0.0;
    public val maxRPM:Double = 0.0;
    public var isSet:Boolean = false;

    public fun setShoot(speed:Double) {
        shootmotor.set(speed);
    }
    public fun toggleMotor(speed:Double) {
        if(isSet) {
            stop();
            isSet = false;
        }else {
            setShoot(speed)
            isSet = true
        }
    }
    public fun stop() {
        shootmotor.stopMotor();
        isSet = false;
    }
    /** This method will be called once per scheduler run  */
    override fun periodic() {
    }

    /** This method will be called once per scheduler run during simulation  */
    override fun simulationPeriodic() {
    }
}