package frc.robot.elevator

import edu.wpi.first.math.controller.PIDController
import frc.lib.swerve.ElevatorIO

class ElevatorIOSparkMax(override val pidController: PIDController) :ElevatorIO {
    
    init{

    }
    override fun updateInputs(inputs: ElevatorIO.ElevatorIOInputs) {

    }

    override fun setBothVolts(volts: Double) {

    }

    override fun reset() {

    }

    override fun stop() {

    }

    override fun setLeftVolts(volts: Double) {

    }

    override fun setRightVolts(volts: Double) {

    }
}