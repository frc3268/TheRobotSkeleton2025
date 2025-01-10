package frc.robot.algaeintake

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

class AlgaeIntakeSubsystem(val io: AlgaeIntakeIO) : SubsystemBase() {
    val inputs = AlgaeIntakeIO.LoggedInputs()

    override fun periodic() {
        io.updateInputs(inputs)
    }

    fun setVoltage(voltage: Double): Command = runOnce { io.setVoltage(voltage) }

    fun stop(): Command = runOnce{ io.stop() }
}