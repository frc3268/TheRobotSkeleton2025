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

    val intakeMotor:CANSparkMax = CANSparkMax(9, CANSparkLowLevel.MotorType.kBrushless)
    val armMotor:CANSparkMax = CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless)
    val armEncoder:RelativeEncoder = armMotor.encoder
    val armPIDController:PIDController = PIDController(0.005,0.0,0.0)
    //todo: extra motor for powered arm

    init {
        armEncoder.positionConversionFactor = 360 / 375 / 2/3 / 1.0
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
            armMotor.set(0.5)
        }.until { getPoweredArmMeasurement().degrees < 5.0 }.andThen(runOnce{stop()})
    }

    fun poweredArmDownCommand():Command{
        return run{
            armMotor.set(-0.5)
        }.until { getPoweredArmMeasurement().degrees > 85.0 }.andThen(runOnce{stop()})
    }

    fun takeInCommand():Command{
        return poweredArmDownCommand().andThen(run{setIntake()}.withTimeout(3.0)).andThen(poweredArmUpCommand())
    }

    fun takeOutCommand():Command{
        return run{setOuttake()}.withTimeout(3.0)
    }

    fun zeroArmEncoderCommand():Command{
        return runOnce{armEncoder.position = 0.0}
    }

    fun getPoweredArmMeasurement() : Rotation2d{
        return armEncoder.position.rotation2dFromDeg()
    }

    override fun periodic() {

    }

    override fun simulationPeriodic() {

    }
}