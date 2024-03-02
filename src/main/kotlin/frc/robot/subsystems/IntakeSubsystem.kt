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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.lib.utils.rotation2dFromDeg

class IntakeSubsystem:SubsystemBase() {
    val intakeMotor = CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless)
    val armMotor = CANSparkMax(9, CANSparkLowLevel.MotorType.kBrushless)
    val armEncoder: RelativeEncoder = armMotor.encoder
    val armPIDController = PIDController(1.0/50,0.0,0.0)
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
        return run {
            armMotor.set(amount)
        }
    }

    fun stopIntake(): Command =
        runOnce { intakeMotor.stopMotor() }

    fun stopArm(): Command =
        runOnce { armMotor.stopMotor() }

    fun stopAllCommand(): Command =
        SequentialCommandGroup(
            stopIntake(),
            stopArm()
        )

    fun basedMomento(): Command =
            runOnce{intakeMotor.set(-1.0)}
    fun setIntake(): Command =
        runOnce { intakeMotor.set(0.3) }

    fun setOuttake(): Command =
        runOnce { intakeMotor.set(-0.3) }

    fun poweredArmUpCommand(): Command =
        run {
            armMotor.set(armPIDController.calculate(getPoweredArmMeasurement().degrees, -5.0))
        }
            .until {getPoweredArmMeasurement().degrees < 0.05 }
            .andThen(stopArm())

    fun poweredArmDownCommand(): Command =
        run { armMotor.set(armPIDController.calculate(getPoweredArmMeasurement().degrees, 270.0)) }
            .until { getPoweredArmMeasurement().degrees > 260.0 }
            .andThen(stopArm())

    fun poweredArmAmpCommand(): Command =
            run {
                armMotor.set(armPIDController.calculate(getPoweredArmMeasurement().degrees, 105.0))
            }
                    .until { getPoweredArmMeasurement().degrees > 100.0 }
                    .andThen(stopArm())

    fun ampCommand(): Command =
            SequentialCommandGroup(
                    poweredArmAmpCommand(),
                    basedMomento(),
                    WaitCommand(2.0),
                    stopIntake(),
                    poweredArmUpCommand()
            )

    fun sourceCommand(): Command =
            SequentialCommandGroup(
                    poweredArmAmpCommand(),
                    setIntake(),
                    WaitCommand(2.0),
                    stopIntake(),
                    poweredArmUpCommand()
            )

    fun takeInCommand(): Command =
        SequentialCommandGroup(
            setIntake(),
            poweredArmDownCommand(),
            WaitCommand(1.0),
            stopIntake(),
            poweredArmUpCommand()
        )

    fun takeOutCommand(): Command =
        SequentialCommandGroup(
            poweredArmUpCommand(),
            setOuttake()
        )

    fun toggleArmCommand(): Command =
        if (getPoweredArmMeasurement().degrees < 100.0)
            poweredArmUpCommand()
        else
            poweredArmDownCommand()

    fun zeroArmEncoderCommand(): Command =
        runOnce { armEncoder.position = 0.0 }

    fun getPoweredArmMeasurement() : Rotation2d =
        armEncoder.position.rotation2dFromDeg()

    override fun periodic() {
        System.out.println(armEncoder.position)

    }

    override fun simulationPeriodic() {

    }
}