package frc.robot.elevator

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.lib.swerve.ElevatorIO
import kotlin.math.abs


class ElevatorSubsystem(val io: ElevatorIO) : SubsystemBase() {
    val inputs = ElevatorIO.LoggedInputs()
    val kg = -0.2

    val cntrl = ProfiledPIDController(io.pidController.p, io.pidController.i, io.pidController.d, TrapezoidProfile.Constraints(80.0, 80.0))

    val troubleshootingtab = Shuffleboard.getTab("Elevator")
    val rightMotorPositionMeters = troubleshootingtab.add("Right Motor Position", 0.0).withPosition(1,0).entry
    val leftMotorPositionMeters = troubleshootingtab.add("Left Motor Position", 0.0).withPosition(1,2).entry
    val elevatorPositionMeters = troubleshootingtab.add("Elevator Position", 0.0).withPosition(1,4).entry

    var gravving = false

    init{

        troubleshootingtab.add("go positive",run{io.setBothVolts(-0.5)}).withWidget(BuiltInWidgets.kCommand)
        troubleshootingtab.add("go negative",run{io.setBothVolts(0.5)}).withWidget(BuiltInWidgets.kCommand)
        troubleshootingtab.add("stop",stop()).withWidget(BuiltInWidgets.kCommand)

        troubleshootingtab.add("go to top",setToPosition(-50.0)).withWidget(BuiltInWidgets.kCommand)

        troubleshootingtab.add("go to half",setToPosition(-36.0)).withWidget(BuiltInWidgets.kCommand)

        troubleshootingtab.add("go to down",setToPosition(0.0)).withWidget(BuiltInWidgets.kCommand)

    }

    override fun periodic() {
        io.updateInputs(inputs)

        // Setting information into the ShuffleBoard for possible debugging.
        rightMotorPositionMeters.setDouble(inputs.rightMotorPositionMeters)
        leftMotorPositionMeters.setDouble(inputs.leftMotorPositionMeters)
        elevatorPositionMeters.setDouble(inputs.elevatorPositionMeters)

        if(inputs.elevatorPositionMeters.toDouble() < -50 || inputs.elevatorPositionMeters > -1 ){
            stop()
        }
        if(gravving == true && inputs.elevatorPositionMeters < 0.5){
            io.setBothVolts(-0.2)
        }
        
    }

    fun setToPosition(setPointMeters: Double): Command =
        runOnce{gravving = false}.andThen(
        run {
            io.setBothVolts(cntrl.calculate(inputs.elevatorPositionMeters, setPointMeters))
        })
            .until { abs(inputs.elevatorPositionMeters - setPointMeters) < 0.5}
            .andThen(
                runOnce{
                    cntrl.reset(inputs.elevatorPositionMeters.toDouble())
                }.andThen(
                    runOnce { gravving = true } ) )

    fun stop(): Command = runOnce { io.stop() }

    fun zero(): Command = runOnce { io.reset() }


}
