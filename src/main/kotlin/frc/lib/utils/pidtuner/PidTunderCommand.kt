package frc.lib.utils.pidtuner

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ExampleSubsystem

/** An example command that uses an example subsystem.  */
class ExampleCommand(private val subsystem: PidTunerSubsystem, val goalDistanceRotations: Double) : CommandBase() {
    var atZero = false
    var lastPos = 0.0
    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem)
    }

    /** Called when the command is initially scheduled.  */
    override fun initialize() {
        atZero = subsystem.getPositionRotations() % 1 == 0.0
        subsystem.resetReadings()
    }

    /** Called every time the scheduler runs while the command is scheduled.  */
    override fun execute() {
        if(!atZero){
            subsystem.set(-0.1)
            atZero = subsystem.getPositionRotations() % 1 == 0.0
            return
        }
        subsystem.addReading(lastPos)
        lastPos = subsystem.getPositionRotations()
        subsystem.setReference(goalDistanceRotations)
    }

    /** Called once the command ends or is interrupted.  */
    override fun end(interrupted: Boolean) {
        //kills the motor
        subsystem.set(0.0)
        subsystem.positiontune(goalDistanceRotations, 1.0)
    }

    /** Returns true when the command should end.  */
    override fun isFinished(): Boolean {
        //tweak this!
        return subsystem.getPowerSent() <= 0.01
    }
}