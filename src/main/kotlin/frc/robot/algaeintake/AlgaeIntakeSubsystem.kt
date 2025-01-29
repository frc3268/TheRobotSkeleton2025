package frc.robot.algaeintake

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase

class AlgaeIntakeSubsystem(val io: AlgaeIntakeIO) : SubsystemBase() {
    val inputs = AlgaeIntakeIO.LoggedInputs()

    override fun periodic() {
        io.updateInputs(inputs)
    }

    fun setVoltage(voltage: Double): Command = runOnce { io.setJointVoltage(voltage) }

    fun stopAll(): Command = runOnce{ io.stop() }
    fun stopJoint(): Command = runOnce{ io.stopJoint() }
    fun stopLeftAndRight(): Command = runOnce{ io.stopLeft() }.alongWith(runOnce { io.stopRight() } )
    fun toggle(): Command = runOnce { io.toggle() }

    fun raiseFromBool(shouldRaise: Boolean): Command = runOnce { io.raiseFromBool(shouldRaise) }

    fun raise(): Command = runOnce { io.setJointVoltage(io.pidController.calculate(0.0, 0.0) * 12.0) }
    fun lower(): Command = runOnce { io.lower() }
}