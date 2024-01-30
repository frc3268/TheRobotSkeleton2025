package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.lib.utils.rotation2dFromDeg
import kotlin.math.abs

class IntakeSubsystem:SubsystemBase() {

    val intakeMotor:CANSparkMax = CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless)
    val armMotor:CANSparkMax = CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless)
    val armEncoder:RelativeEncoder = armMotor.encoder
    val armPIDController:PIDController = PIDController(0.0,0.0,0.0)
    //todo: extra motor for powered arm

    init {
        //todo:fix!
        armEncoder.positionConversionFactor = 1.0
    }

    fun stop() {
        intakeMotor.stopMotor()
    }

    fun setIntake(){
        intakeMotor.set(0.3)
    }

    fun setOuttake(){
        intakeMotor.set(-.3)
    }

    fun poweredArmUpCommand():Command{
        return run{
            armMotor.set(armPIDController.calculate(getPoweredArmMeasurement().degrees, 50.0))
        }.until{abs(armPIDController.calculate(getPoweredArmMeasurement().degrees, 50.0) )< 0.05}
    }

    fun poweredArmDownCommand():Command{
        return run{
            armMotor.set(armPIDController.calculate(getPoweredArmMeasurement().degrees, 10.0))
        }.until{abs(armPIDController.calculate(getPoweredArmMeasurement().degrees, 10.0) )< 0.05}
    }

    fun takeInCommand():Command{
        return poweredArmDownCommand().andThen(run{setIntake()}.withTimeout(3.0)).andThen(poweredArmUpCommand())
    }

    fun getPoweredArmMeasurement() : Rotation2d{
        return armEncoder.position.rotation2dFromDeg()
    }

    override fun periodic() {

    }

    override fun simulationPeriodic() {

    }
}