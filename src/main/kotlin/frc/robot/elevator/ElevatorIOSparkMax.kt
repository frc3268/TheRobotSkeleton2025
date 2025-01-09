package frc.robot.elevator

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.PIDController
import frc.lib.swerve.ElevatorIO

class ElevatorIOSparkMax(override val pidController: PIDController) :ElevatorIO {
    val leftMotor = CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless)
    val rightMotor = CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless)

    val leftEncoder = leftMotor.encoder
    val rightEncoder = rightMotor.encoder

    init{
        leftEncoder.positionConversionFactor = 0.0
        rightEncoder.positionConversionFactor = 0.0

    }
    override fun updateInputs(inputs: ElevatorIO.ElevatorIOInputs) {
        //this forumla may need to me changed to reflect the reality
        inputs.elevatorPositionMeters = (leftEncoder.position + rightEncoder.position) / 2
        inputs.rightMotorPositionMeters = rightEncoder.position
        inputs.leftMotorPositionMeters = leftEncoder.position
        inputs.rightMotorCurrentAmps = doubleArrayOf(rightMotor.outputCurrent)
        inputs.leftMotorCurrentAmps = doubleArrayOf(leftMotor.outputCurrent)
        inputs.rightMotorAppliedVolts = rightMotor.busVoltage * rightMotor.appliedOutput
        inputs.leftMotorAppliedVolts = leftMotor.busVoltage * leftMotor.appliedOutput
        inputs.rightMotorVelocityMetersPerSec = rightEncoder.velocity
        inputs.leftMotorVelocityMetersPerSec = leftEncoder.velocity
    }

    override fun setBothVolts(volts: Double) {
        rightMotor.setVoltage(volts)
        leftMotor.setVoltage(volts)
    }

    override fun reset() {
        rightEncoder.position = 0.0
        leftEncoder.position = 0.0
    }

    override fun stop() {
        rightMotor.stopMotor()
        leftMotor.stopMotor()
    }

    override fun setLeftVolts(volts: Double) {
        leftMotor.setVoltage(volts)
    }

    override fun setRightVolts(volts: Double) {
        rightMotor.setVoltage(volts)
    }
}