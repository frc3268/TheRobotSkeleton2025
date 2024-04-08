package frc.robot.subsystems

import com.revrobotics.*
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.*
import frc.lib.rotation2dFromDeg

class IntakeSubsystem: SubsystemBase() {
    val intakeMotor = CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless)
    val armMotor = CANSparkMax(9, CANSparkLowLevel.MotorType.kBrushless)
    val armEncoder: RelativeEncoder = armMotor.encoder
    val armPIDController = PIDController(1.0/50,0.0,0.0)

    val shuffleboardTab = Shuffleboard.getTab("General")

    companion object {
        const val INTAKE_SPEED = 0.3
        const val OUTTAKE_ADJUST_SPEED = -0.3
        const val OUTTAKE_SPEED = -0.9
        const val SHOOT_AMP_SPEED = -1.0
        //based momento...

        const val UP_ANGLE = 0.05
        const val DOWN_ANGLE = 270.0
    }

    init {
        armEncoder.positionConversionFactor = 3 / 2 / 1.0
    }

    fun stopIntake(): Command =
        runOnce { intakeMotor.stopMotor() }

    fun stopArm(): Command =
        runOnce { armMotor.stopMotor() }

    /**
     * Stops both the arm and intake gears immediately.
     */
    fun stopAllCommand(): Command =
        SequentialCommandGroup(
            stopIntake(),
            stopArm()
        )

    /**
     * Runs the intake gears at [speed].
     */
    fun runIntakeAtSpeed(speed: Double): Command =
        runOnce { intakeMotor.set(speed) }

    fun armUpCommand(): Command =
        run { armMotor.set(armPIDController.calculate(getArmPosition().degrees, UP_ANGLE-5.0)) }
            .until { getArmPosition().degrees < UP_ANGLE }
            .andThen(stopArm())

    fun armDownCommand(): Command =
        run { armMotor.set(armPIDController.calculate(getArmPosition().degrees, DOWN_ANGLE+5.0)) }
            .until { getArmPosition().degrees > DOWN_ANGLE }
            .andThen(stopArm())

    fun takeInCommand(): Command =
        SequentialCommandGroup(
            runIntakeAtSpeed(INTAKE_SPEED),
            armDownCommand(),
            WaitCommand(0.8),
        )

    fun takeOutCommand(): Command =
        SequentialCommandGroup(
            armUpCommand(),
            runIntakeAtSpeed(OUTTAKE_SPEED)
        )

    fun zeroArmEncoderCommand(): Command =
        runOnce { armEncoder.position = 0.0 }

    fun getArmPosition() : Rotation2d =
        armEncoder.position.rotation2dFromDeg()

    override fun periodic() {
        // Stop arm guard in case it screws itself over
        if (getArmPosition().degrees >= 290.0) armUpCommand()
        else if (getArmPosition().degrees <= -10.0)
            armDownCommand()

    }

    override fun simulationPeriodic() {
    }
}