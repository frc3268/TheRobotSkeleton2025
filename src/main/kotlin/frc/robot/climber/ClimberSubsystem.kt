package frc.robot.climber

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.abs
import frc.robot.Constants


class ClimberSubsystem(val io: ClimberIO) : SubsystemBase() {
    val inputs = ClimberIO.LoggedInputs()
    val kg = 0.0

    // Get some trouble - shooting information to the ShuffleBoard
    val troubleshootingtab = Shuffleboard.getTab("climn")
    val rightMotorPositionDegrees = troubleshootingtab.add("Right Motor Position", 0.0).withPosition(4,0).entry
    val rightMotorAppliedVolts = troubleshootingtab.add("Right Motor Applied Volts", 0.0).withPosition(4, 1).entry
    val leftMotorPositionDegrees = troubleshootingtab.add("Left Motor Position", 0.0).withPosition(4, 2).entry
    val leftMotorAppliedVolts = troubleshootingtab.add("Left Motor Applied Volts", 0.0).withPosition(4, 3).entry
    init {
        troubleshootingtab.add("turn it", setToPosition(3.0)).withWidget(BuiltInWidgets.kCommand)

    }

    override fun periodic() {
        io.updateInputs(inputs)
        leftMotorPositionDegrees.setDouble(inputs.motorPositionDegrees)
    }

    fun setToPosition(setPointDegrees: Double): Command =
        run {
            io.setBothVolts(0.4 * 12.0)
        }
            // .until { abs(inputs.motorPositionDegrees - setPointDegrees) < 1.0 }

    fun stop(): Command = runOnce { io.stop() }

    fun zero(): Command = runOnce { io.reset() }
}