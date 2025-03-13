package frc.robot.algaeintake

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import frc.lib.rotation2dFromDeg
import kotlin.math.abs


//algae has 3 motors - one for arm, two for wheels. main and rev are wheel motors, rev always goes reverse direction
class AlgaeIntakeSubsystem(val io: AlgaeIntakeIO) : SubsystemBase() {
    val inputs = AlgaeIntakeIO.LoggedInputs()

    val troubleshootingTab = Shuffleboard.getTab("Algae")
    val jointAngleEntry = troubleshootingTab.add("Joint Angle", 0.0).withPosition(2, 0).entry
    val jointVelocityMetersPerSecEntry = troubleshootingTab.add("Joint Velocity MPS", 0.0).withPosition(2, 1).entry
    val mainVelocityMetersPerSecEntry = troubleshootingTab.add("Main Velocity MPS", 0.0).withPosition(2, 2).entry
    val revVelocityMetersPerSecEntry = troubleshootingTab.add("Reverse Velocity MPS", 0.0).withPosition(2, 3).entry

    val cntrl = ProfiledPIDController(io.pidController.p, io.pidController.i, io.pidController.d, TrapezoidProfile.Constraints(60.0, 60.0))

    var setpoint = 0.0
    var stopped = false
    var startedSpinning = false

    init {

        io.resetJointEncoder()
        troubleshootingTab.add("up", raise()).withWidget(BuiltInWidgets.kCommand)
        troubleshootingTab.add("down", lower()).withWidget(BuiltInWidgets.kCommand)
        troubleshootingTab.add("half", half()).withWidget(BuiltInWidgets.kCommand)
        troubleshootingTab.add("stop", stopJoint()).withWidget(BuiltInWidgets.kCommand)
        troubleshootingTab.add("intake", intake()).withWidget(BuiltInWidgets.kCommand)
        troubleshootingTab.add("outtake", outtake()).withWidget(BuiltInWidgets.kCommand)
        troubleshootingTab.add("stopwheels", stopWheels()).withWidget(BuiltInWidgets.kCommand)
    }
    
    override fun periodic() {
        io.updateInputs(inputs)

        // Debug stuff I guess
        jointAngleEntry.setDouble(inputs.jointAngle.degrees)
        jointVelocityMetersPerSecEntry.setDouble(inputs.jointVelocityMetersPerSec.toDouble())
        mainVelocityMetersPerSecEntry.setDouble(inputs.mainVelocityMetersPerSec.toDouble())
        revVelocityMetersPerSecEntry.setDouble(inputs.revVelocityMetersPerSec.toDouble())

        if(!stopped){
            io.setJointVoltage(io.pidController.calculate(inputs.jointAngle.degrees, setpoint))
        }

    }
    fun intake(): Command = run{io.setMainAndRevVoltage(-0.3 * 12.0)}
        .withTimeout(1.0)
        .andThen(run{})
        .until { abs(inputs.mainVelocityMetersPerSec) > 3000 }
        .andThen(runOnce { startedSpinning = true })
        .andThen(run{})
        .until { abs(inputs.mainVelocityMetersPerSec) < 2950 && startedSpinning == true}
        .andThen(stopWheels())
        .andThen(runOnce{startedSpinning = false})



    fun outtake(): Command = run{io.setMainAndRevVoltage(0.3 * 12.0)}
        .withTimeout(1.0)
        .andThen(run{})
        .until { abs(inputs.mainVelocityMetersPerSec) > 2900 }
        .andThen(stopWheels())




    fun stopAll(): Command = runOnce{
        stopped = true
        io.stop()
    }
    fun stopJoint(): Command = runOnce{
        stopped = true
        io.stopJoint()
    }
    fun stopWheels(): Command = runOnce{ io.stopMain() }.andThen(runOnce { io.stopRev()})
    fun toggle(): Command = if (inputs.jointAngle.degrees > 0.0) {
        raise()
    } else if (inputs.jointAngle.degrees < 0.0) {
        lower()
    } else {
        runOnce{}
    }

    fun raise(): Command =
        runOnce{stopped = true}.andThen(
        run {
        io.setJointVoltage(cntrl.calculate(inputs.jointAngle.degrees, 10.0))
    }.until { (inputs.jointAngle - 10.0.rotation2dFromDeg()).degrees < 2.0 }.andThen(
        runOnce{
            cntrl.reset(inputs.jointAngle.degrees)
        }.andThen(
            runOnce {
                setpoint = 10.0
                stopped = false
            } ) ))

    fun lower(): Command =
        runOnce {
            stopped = true
        }
            .andThen(run {
        io.setJointVoltage(cntrl.calculate(inputs.jointAngle.degrees, -110.0))
    }.until { (inputs.jointAngle - (-110.0).rotation2dFromDeg()).degrees < 2.0 }.andThen(
        runOnce{
            cntrl.reset(inputs.jointAngle.degrees)
        }.andThen(
            runOnce {
                setpoint = -110.0
                stopped = false } ) ))


    fun half(): Command =
        runOnce {
            stopped = true
        }.andThen(
        run{
        io.setJointVoltage(cntrl.calculate(inputs.jointAngle.degrees, -95.0))
        }.until { (inputs.jointAngle - (-80.0).rotation2dFromDeg()).degrees < 2.0 }.andThen(
        runOnce{
            cntrl.reset(inputs.jointAngle.degrees)
        }.andThen(
            runOnce {
                setpoint = -95.0
                stopped = false } ) ))

    fun dropAlgae() : Command = runOnce {
        outtake()
        lower()
    }
}