package frc.robot.coralintake

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.Constants
import kotlin.math.abs

class CoralIntakeSubsystem(val io: CoralIntakeIO) : SubsystemBase() {
    val inputs = CoralIntakeIO.LoggedInputs()

    // Wtf, why does this not work?
    val troubleshootingTab = Shuffleboard.getTab("Coral")
    val jointAngleEntry = troubleshootingTab.add("Joint Angle", 0.0).withPosition(3, 0).withWidget(BuiltInWidgets.kGyro).entry
    val jointVelocityRotPerMinEntry = troubleshootingTab.add("Joint Velocity RPM", 0.0).withPosition(3, 1).entry
    val intakeVelocityRotPerMinEntry = troubleshootingTab.add("Intake Velocity RPM", 0.0).withPosition(3, 2).entry
    var setpoint = 0.0
    var stopped = false
    var initvel = 0.0

    init{
        troubleshootingTab.add("outtake",outtake()).withWidget(BuiltInWidgets.kCommand)
        troubleshootingTab.add("intake",intake()).withWidget(BuiltInWidgets.kCommand)
        troubleshootingTab.add("go up",run{io.setJointVoltage(0.5)}).withWidget(BuiltInWidgets.kCommand)
        troubleshootingTab.add("reset", runOnce { io.reset() }).withWidget(BuiltInWidgets.kCommand)

        troubleshootingTab.add("stop", runOnce { stop() }).withWidget(BuiltInWidgets.kCommand)
        troubleshootingTab.add("go to score",raiseToScore()).withWidget(BuiltInWidgets.kCommand)
        troubleshootingTab.add("go to source",raiseToIntake()).withWidget(BuiltInWidgets.kCommand)
    }

    override fun periodic() {
        io.updateInputs(inputs)
        jointAngleEntry.setDouble(inputs.jointAngle.degrees)
        jointVelocityRotPerMinEntry.setDouble(inputs.jointVelocityRPM)
        intakeVelocityRotPerMinEntry.setDouble(inputs.intakeVelocityRPM)

        if(!stopped){
            io.setJointVoltage(io.pidController.calculate(inputs.jointAngle.degrees, setpoint))
        }

    }

    fun intake(): Command = run{io.setIntakeVoltage(-0.1 * 12.0)}.until { abs(inputs.intakeVelocityRPM) > 300.0 }.andThen(
        WaitCommand(2.0)).andThen(stopIntake())

    fun outtake(): Command = run{io.setIntakeVoltage(0.1 * 12.0)}.until { abs(inputs.intakeVelocityRPM) > 300.0 }

    fun raiseToScore(): Command = runOnce {
        stopped = false
        setpoint = -40.0
    }

    fun raiseToIntake(): Command = runOnce {
        setpoint = -30.0
    }

    fun lower(): Command = runOnce {
        stopped = false
        setpoint = 0.0
    }

    fun stop(): Command = runOnce{
        io.stop()
        stopped = true
    }

    fun reset():Command = runOnce{stopped = true}.andThen(runOnce{
        io.reset()
        io.stopJoint()
    })

    fun stopJoint(): Command = runOnce{ io.stopJoint() }
    fun stopIntake(): Command = runOnce{ io.stopIntake() }


}
