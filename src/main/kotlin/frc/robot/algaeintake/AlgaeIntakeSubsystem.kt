package frc.robot.algaeintake

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase

class AlgaeIntakeSubsystem(val io: AlgaeIntakeIO) : SubsystemBase() {
    val inputs = AlgaeIntakeIO.LoggedInputs()

    override fun periodic() {
        io.updateInputs(inputs)
    }

    fun intake():Command = run { io.setLeftAndRightVoltage(0.3 * 12.0) }.withTimeout(1.5)

    fun outtake():Command = run { io.setLeftAndRightVoltage(-0.3 * 12.0) }.withTimeout(1.5)

    fun stopAll(): Command = runOnce{ io.stop() }
    fun stopJoint(): Command = runOnce{ io.stopJoint() }
    fun stopLeftAndRight(): Command = runOnce{ io.stopRight() }.alongWith(runOnce { io.stopLeft()})
    fun toggle(): Command = if (inputs.jointAngle.degrees > 0.0) {
        raise()
    } else if (inputs.jointAngle.degrees < 0.0) {
        lower()
    } else {
        runOnce{}
    }


    fun raise(): Command = runOnce {
        io.setJointVoltage(io.pidController.calculate(0.0, 0.0))
    }.until { inputs.jointAngle.degrees > 0.0 }

    fun lower(): Command = runOnce {
        io.setJointVoltage(io.pidController.calculate(0.0, 0.0))
    }.until { inputs.jointAngle.degrees < 0.0 }
}