package frc.robot.elevator

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.lib.swerve.ElevatorIO
import kotlin.math.abs
import frc.robot.Constants


class ElevatorSubsystem(val io: ElevatorIO) : SubsystemBase() {
    val inputs = ElevatorIO.LoggedInputs()
    val kg = 0.0

    val troubleshootingtab = Shuffleboard.getTab(Constants.TROUBLESHOOTING_TAB)
    val rightMotorPositionMeters = troubleshootingtab.add("Right Motor Position", 0.0).withPosition(1,0).entry
    val rightMotorAppliedVolts = troubleshootingtab.add("Right Motor Applied Volts", 0.0).withPosition(1, 1).entry
    val leftMotorPositionMeters = troubleshootingtab.add("Left Motor Position", 0.0).withPosition(1,2).entry
    val leftMotorAppliedVolts = troubleshootingtab.add("Left Motor Applied Volts", 0.0).withPosition(1, 3).entry
    val elevatorPositionMeters = troubleshootingtab.add("Elevator Position", 0.0).withPosition(1,4).entry

    override fun periodic() {
        io.updateInputs(inputs)

        // Setting information into the ShuffleBoard for possible debugging.
        rightMotorPositionMeters.setDouble(inputs.rightMotorPositionMeters)
        rightMotorAppliedVolts.setDouble(inputs.rightMotorAppliedVolts)
        leftMotorPositionMeters.setDouble(inputs.leftMotorPositionMeters)
        leftMotorAppliedVolts.setDouble(inputs.leftMotorAppliedVolts)
        elevatorPositionMeters.setDouble(inputs.elevatorPositionMeters)
        
    }

    fun setToPosition(setPointMeters: Double): Command =
        run {
            io.setBothVolts(io.pidController.calculate(inputs.elevatorPositionMeters, setPointMeters) * 12.0)
        }
            .until { abs(inputs.elevatorPositionMeters - setPointMeters) < 0.01 }
            .andThen(
                if (setPointMeters > 0) {
                    run { io.setBothVolts(kg * 12.0) }
                } else {
                    stop()
                })

    fun stop(): Command = runOnce { io.stop() }

    fun zero(): Command = runOnce { io.reset() }
}
