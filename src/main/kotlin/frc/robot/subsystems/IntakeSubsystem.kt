package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.lib.utils.rotation2dFromDeg

class IntakeSubsystem:SubsystemBase() {

    val intakeMotor:CANSparkMax = CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless)
    val armMotor:CANSparkMax = CANSparkMax(9, CANSparkLowLevel.MotorType.kBrushless)
    val armEncoder:RelativeEncoder = armMotor.encoder
    val armPIDController:PIDController = PIDController(0.005,0.0,0.0)
    //todo: extra motor for powered arm
    val ShuffleboardTab:ShuffleboardTab = Shuffleboard.getTab("intake")
    val intakeArmEncoderEntry: GenericEntry = ShuffleboardTab.add("Angle Encoder(ARM)", 0.0).entry

    init {
        armEncoder.positionConversionFactor = 3 / 2 / 1.0
        ShuffleboardTab.add("up", poweredArmUpCommand()).withWidget(BuiltInWidgets.kCommand)
        ShuffleboardTab.add("down", poweredArmDownCommand()).withWidget(BuiltInWidgets.kCommand)
        ShuffleboardTab.add("stop", stopAllCommand()).withWidget(BuiltInWidgets.kCommand)
        ShuffleboardTab.add("in", takeInCommand()).withWidget(BuiltInWidgets.kCommand)
        ShuffleboardTab.add("out", takeOutCommand()).withWidget(BuiltInWidgets.kCommand)
    }

    fun armsetCommand(amount: Double): Command {
        stopArm()
        return run{
            armMotor.set(amount)
        }
    }

    fun stopIntake() {
        intakeMotor.stopMotor()
    }
    fun stopArm() {
        armMotor.stopMotor()
    }

    fun stopAllCommand():Command{
        return runOnce{
            stopIntake()
            stopArm()
        }
    }

    fun setIntake(){
        intakeMotor.set(0.3)
    }

    fun setOuttake(){
        intakeMotor.set(-.3)
    }

    fun poweredArmUpCommand():Command{
        return run{
            armMotor.set(-0.3)
        }.until { getPoweredArmMeasurement().degrees < 5.0 }.andThen(stopAllCommand())
    }

    fun poweredArmDownCommand():Command{
        return run{
            armMotor.set(0.3)
        }.until { getPoweredArmMeasurement().degrees > 265.0 }.andThen(stopAllCommand())
    }

    fun takeInCommand():Command{
        return poweredArmDownCommand().andThen(run{setIntake()}.withTimeout(3.0)).andThen(runOnce { stopIntake() }).andThen(poweredArmUpCommand())
    }

    fun takeOutCommand():Command{
        return run{setOuttake()}.withTimeout(3.0).andThen(runOnce{stopIntake()})
    }

    fun zeroArmEncoderCommand():Command{
        return runOnce{armEncoder.position = 0.0}
    }

    fun getPoweredArmMeasurement() : Rotation2d{
        return armEncoder.position.rotation2dFromDeg()
    }

    override fun periodic() {
        System.out.println(armEncoder.position)

    }

    override fun simulationPeriodic() {

    }
}