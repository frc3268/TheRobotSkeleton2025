package frc.lib.motor

import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.Slot1Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.DCMotorSim


class KrakenSimMotor(
    override val id: Int,
    override var inverse: Boolean = false,
    var positionPIDController: PIDController,
    var velocityPIDController: PIDController,
) : Motor {

    val motor = TalonFX(id, "rio")
    val motorConfig = TalonFXConfiguration()

    var positionSlot = Slot0Configs()
    var velocitySlot = Slot1Configs()
    val motorDC = DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            0.005, // I don't know what these funny numbers mean. TODO: Can someone please fill these in?
            1.0),
        DCMotor.getNEO(1),
    )


    init{

        if (inverse) {
            motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        }
        else {
            motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive
        }

        positionSlot.kP = positionPIDController.p
        positionSlot.kI = positionPIDController.i
        positionSlot.kD = positionPIDController.d

        velocitySlot.kP = velocityPIDController.p
        velocitySlot.kI = velocityPIDController.i
        velocitySlot.kD = velocityPIDController.d

        motorConfig.Feedback.SensorToMechanismRatio = 0.0

        configure()
    }
    override fun configure() {
        motor.configurator.apply(motorConfig)
        motor.configurator.apply(positionSlot)
        motor.configurator.apply(velocitySlot)
    }

    override fun setVoltage(voltage: Double){
        motor.setVoltage(voltage)
    }

    override fun setPosition(position: Double) {
        motor.setPosition(position)
    }

    override fun setVelocity(velocity: Double) {
        val request = VelocityVoltage(velocity).withSlot(1);
        motor.setControl(request)
    }

    override fun getVelocityRPMMeasurement(): Double {
        return motor.velocity.valueAsDouble
    }

    override fun getAppliedVoltage(): Double {
        return motor.motorVoltage.valueAsDouble
    }

    override fun getPositionDegreeMeasurement(): Double {
        return getAppliedVoltage() / 360
    }

    override fun getCurrentAmps(): DoubleArray {
        return doubleArrayOf(motor.statorCurrent.valueAsDouble)
    }

    override fun stop() {
        motor.stopMotor()
    }

    override fun close() {
        motor.close()
    }

    override fun reset() {
        //not totally sure if this works as intended
        //as intended means that it just changes the value reported by encoder
        motor.setPosition(Angle.ofRelativeUnits(0.0, Units.Degree))
    }

    override fun simulationPeriodic() {
        var motorSim = motor.simState
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage())
        var motorVoltage = motorSim.motorVoltage;

        // Use the motor voltage to calculate new position and velocity
        // Using WPILib's DCMotorSim class for physics simulation
        motorDC.setInputVoltage(motorVoltage);
        motorDC.update(0.020); // Assume 20 ms loop time

        // I don't know what these funny numbers mean. TODO: Can someone please fill these in?
        motorSim.setRawRotorPosition(
            1 * motorDC.angularPositionRotations
        );
        motorSim.setRotorVelocity(
            1 * motorDC.angularVelocityRadPerSec
        );
    }
}