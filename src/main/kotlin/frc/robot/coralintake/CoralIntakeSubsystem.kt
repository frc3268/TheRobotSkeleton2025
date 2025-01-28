package frc.robot.coralintake

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

class CoralIntakeSubsystem(val io: CoralIntakeIO) : SubsystemBase() {
    val inputs = CoralIntakeIO.LoggedInputs()

    override fun periodic() {
        io.updateInputs(inputs)
    }

    fun setWheelVoltage(voltage: Double): Command = runOnce { io.setWheelVoltage(voltage) }

    fun stop(): Command = runOnce{ io.stop() }
}
