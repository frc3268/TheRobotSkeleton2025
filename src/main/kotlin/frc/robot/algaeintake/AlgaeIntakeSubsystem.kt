package frc.robot.algaeintake

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase

class AlgaeIntakeSubsystem(val io: AlgaeIntakeIO) : SubsystemBase() {
    val inputs = AlgaeIntakeIO.LoggedInputs()

    override fun periodic() {
        io.updateInputs(inputs)
    }

    fun setVoltage(voltage: Double): Command = runOnce { io.setJointVoltage(voltage) }

    fun setLeftAndRightVoltage(voltage: Double) { io.setLeftAndRightVoltage(voltage) }
    fun setLeftVoltage(voltage: Double) { io.setLeftVoltage(voltage) }
    fun setRightVoltage(voltage: Double) { io.setRightVoltage(voltage) }

    fun stopAll(): Command = runOnce{ io.stopAll() }
    fun stopJoint(): Command = runOnce{ io.stopJoint() }
    fun stopLeftAndRight(): Command = runOnce{ io.stopLeftAndRight() }
    fun toggle(): Command = runOnce { io.toggle() }

    fun raiseFromBool(shouldRaise: Boolean): Command = runOnce { io.raiseFromBool(shouldRaise) }

    fun raise(): Command = runOnce { io.raise() }
    fun lower(): Command = runOnce { io.lower() }
}