package frc.robot.algaeintake

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase

class AlgaeIntakeSubsystem(val io: AlgaeIntakeIO) : SubsystemBase() {
    val inputs = AlgaeIntakeIO.LoggedInputs()

    override fun periodic() {
        io.updateInputs(inputs)
    }

    fun setVoltage(voltage: Double): Command = runOnce { io.setVoltage(voltage) }

    fun stopAll(): Command = runOnce{ io.stop() }


    fun toggle(): Command = runOnce {

    }

    fun raiseFromBool(shouldRaise: Boolean): Command = runOnce {
        if (shouldRaise) {
            raise()
        }
        else {
            lower()
        }
    }

    fun raise(): Command = runOnce {}
    fun lower(): Command = runOnce {}
}