package frc.robot.subsystems

import com.revrobotics.*
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.*
import frc.lib.utils.rotation2dFromDeg

class IntakeSubsystem: SubsystemBase() {
    val intakeMotor = CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless)
    val armMotor = CANSparkMax(9, CANSparkLowLevel.MotorType.kBrushless)
    val armEncoder: RelativeEncoder = armMotor.encoder
    val armPIDController = PIDController(1.0/50,0.0,0.0)

    companion object {
        const val INTAKE_SPEED = 0.3
        const val OUTTAKE_ADJUST_SPEED = -0.3
        const val OUTTAKE_SPEED = -0.9
        const val SHOOT_AMP_SPEED = -1.0
        //based momento...

        const val UP_ANGLE = 0.05
        const val DOWN_ANGLE = 275.0
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
        run { armMotor.set(armPIDController.calculate(getArmPosition().degrees, UP_ANGLE)) }
            .until { getArmPosition().degrees < UP_ANGLE }
            .andThen(stopArm())

    fun armDownCommand(): Command =
        run { armMotor.set(armPIDController.calculate(getArmPosition().degrees, DOWN_ANGLE)) }
            .until { getArmPosition().degrees > DOWN_ANGLE }
            .andThen(stopArm())

    fun toggleArmCommand(): Command =
        if (getArmPosition().degrees < 100.0)
            armUpCommand()
        else
            armDownCommand()

    /**
     * Sets the arm to the amp shoot or source intake angle, which are the same.
     */
    fun armToAmpAngleCommand(): Command =
        run { armMotor.set(armPIDController.calculate(getArmPosition().degrees, 105.0)) }
            .until { getArmPosition().degrees > 100.0 }
            .andThen(stopArm())

    /**
     * Brings the arm up to the amp angle and shoots the note into the amp.
     */
    fun ampCommand(): Command =
        SequentialCommandGroup(
            armToAmpAngleCommand(),
            runIntakeAtSpeed(SHOOT_AMP_SPEED),
            WaitCommand(2.0),
            stopIntake(),
            armUpCommand()
        )

    /**
     * Brings the arm up to the source intake angle and then intakes a note.
     */
    fun armUpAndIntakeCommand(): Command =
        SequentialCommandGroup(
            armToAmpAngleCommand(),
            runIntakeAtSpeed(INTAKE_SPEED),
            WaitCommand(0.1),
            stopIntake(),
            armUpCommand()
        )

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

    fun runOnceIntake(): Command =
        runIntakeAtSpeed(INTAKE_SPEED).andThen(stopIntake())

    fun runOnceOuttake(): Command =
        runIntakeAtSpeed(OUTTAKE_ADJUST_SPEED).andThen(stopIntake())

    fun zeroArmEncoderCommand(): Command =
        runOnce { armEncoder.position = 0.0 }

    fun getArmPosition() : Rotation2d =
        armEncoder.position.rotation2dFromDeg()

    override fun periodic() {
        println("Arm angle: " + getArmPosition().degrees)

        // Stop arm guard in case it screws itself over
        if (getArmPosition().degrees !in -4.0..290.0)
            stopArm()
    }

    override fun simulationPeriodic() {
    }
}