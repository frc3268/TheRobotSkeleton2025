package frc.robot.coralintake

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants

class CoralIntakeSubsystem(val io: CoralIntakeIO) : SubsystemBase() {
    val inputs = CoralIntakeIO.LoggedInputs()

    val troubleshootingTab = Shuffleboard.getTab(Constants.TROUBLESHOOTING_TAB)
    val jointAngleEntry = troubleshootingTab.add("Joint Angle", 0.0).withPosition(3, 0).entry
    val jointVelocityRotPerMinEntry = troubleshootingTab.add("Joint Velocity RPM", 0.0).withPosition(3, 1).entry
    val intakeVelocityRotPerMinEntry = troubleshootingTab.add("Intake Velocity RPM", 0.0).withPosition(3, 2).entry

    override fun periodic() {
        io.updateInputs(inputs)
        jointAngleEntry.setDouble(inputs.jointAngle.degrees)
        jointVelocityRotPerMinEntry.setDouble(inputs.jointVelocityRPM)
        intakeVelocityRotPerMinEntry.setDouble(inputs.intakeVelocityRPM)
    }

    fun intake(): Command = run{io.setIntakeVoltage(0.3 * 12.0)}.withTimeout(1.5)

    fun outtake(): Command = run{io.setIntakeVoltage(-0.3 * 12.0)}.withTimeout(1.5)

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
    fun stopIntake(): Command = runOnce{ io.stopIntake() }
}
