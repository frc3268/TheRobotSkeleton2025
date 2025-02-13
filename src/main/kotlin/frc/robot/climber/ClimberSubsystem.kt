package frc.robot.climber

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.abs
import frc.robot.Constants


class ClimberSubsystem(val io: ClimberIO) : SubsystemBase() {
    val inputs = ClimberIO.LoggedInputs()
    val kg = 0.0

    val troubleshootingtab = Shuffleboard.getTab(Constants.TROUBLESHOOTING_TAB)
    val rightmotorposition = troubleshootingtab.add("Right Motor Position", 0.0).withPosition(1,0).entry
    init {

    }

    override fun periodic() {
        io.updateInputs(inputs)
    }

    fun setToPosition(setPointDegrees: Double): Command =
        run {
            io.setBothVolts(io.pidController.calculate(inputs.climberPositionDegrees, setPointDegrees) * 12.0)
        }
            .until { abs(inputs.climberPositionDegrees - setPointDegrees) < 10.0 }

    fun stop(): Command = runOnce { io.stop() }

    fun zero(): Command = runOnce { io.reset() }
}