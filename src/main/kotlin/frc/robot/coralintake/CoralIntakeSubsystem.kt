package frc.robot.coralintake

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

class CoralIntakeSubsystem(val io: CoralIntakeIO) : SubsystemBase() {
    val inputs = CoralIntakeIO.LoggedInputs()

    override fun periodic() {
        io.updateInputs(inputs)
    }

    fun intake(): Command = run{io.setWheelVoltage(0.3 * 12.0)}.withTimeout(1.5)

    fun outtake(): Command = run{io.setWheelVoltage(-0.3 * 12.0)}.withTimeout(1.5)

    fun raiseToScore(): Command = runOnce {
        io.setJointVoltage(io.pidController.calculate(0.0, 0.0))
    }.until { inputs.jointAngle.degrees > 0.0 }

    fun raiseToIntake(): Command = runOnce {
        io.setJointVoltage(io.pidController.calculate(0.0, 0.0))
    }.until { inputs.jointAngle.degrees > 0.0 }

    fun lower(): Command = runOnce {
        io.setJointVoltage(io.pidController.calculate(0.0, 0.0))
    }.until { inputs.jointAngle.degrees < 0.0 }

    fun stop(): Command = runOnce{ io.stop() }
    fun stopJoint(): Command = runOnce{ io.stopJoint() }
    fun stopWheel(): Command = runOnce{ io.stopWheel() }
}
