package frc.robot.coralintake

import com.revrobotics.sim.SparkMaxSim
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.simulation.RoboRioSim


class CoralIntakeIOSparkMaxSim : CoralIntakeIO {
    override val pidController: PIDController = PIDController(0.0,0.0,0.0)

    var neoGearbox: DCMotor = DCMotor.getNeo550(1)

    val jointMotor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)
    var jointSim: SparkMaxSim = SparkMaxSim(jointMotor, neoGearbox)
    val jointConfig = SparkMaxConfig()

    val intakeMotor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)
    var intakeSim: SparkMaxSim = SparkMaxSim(intakeMotor, neoGearbox)
    val intakeConfig = SparkMaxConfig()


    init {
        jointConfig.encoder.positionConversionFactor(0.0)
        intakeConfig.encoder.positionConversionFactor(0.0)

        intakeMotor.configure(intakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        jointMotor.configure(jointConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    override fun setIntakeVoltage(voltage: Double) {
        intakeMotor.setVoltage(voltage)
    }

    override fun setJointVoltage(volatge: Double) {
        jointMotor.setVoltage(volatge)
    }

    override fun stopJoint() {
        jointMotor.stopMotor()
    }

    override fun stopIntake() {
        intakeMotor.stopMotor()
    }

    override fun stop() {
        stopIntake()
        stopJoint()
    }

    override fun updateInputs(inputs: CoralIntakeIO.Inputs) {

        /*
        ░░░░░▄▄▄▄▀▀▀▀▀▀▀▀▄▄▄▄▄▄░░░░░░░
        ░░░░░█░░░░▒▒▒▒▒▒▒▒▒▒▒▒░░▀▀▄░░░░
        ░░░░█░░░▒▒▒▒▒▒░░░░░░░░▒▒▒░░█░░░
        ░░░█░░░░░░▄██▀▄▄░░░░░▄▄▄░░░░█░░
        ░▄▀▒▄▄▄▒░█▀▀▀▀▄▄█░░░██▄▄█░░░░█░
        █░▒█▒▄░▀▄▄▄▀░░░░░░░░█░░░▒▒▒▒▒░█
        █░▒█░█▀▄▄░░░░░█▀░░░░▀▄░░▄▀▀▀▄▒█
        ░█░▀▄░█▄░█▀▄▄░▀░▀▀░▄▄▀░░░░█░░█░
        ░░█░░░▀▄▀█▄▄░█▀▀▀▄▄▄▄▀▀█▀██░█░░
        ░░░█░░░░██░░▀█▄▄▄█▄▄█▄████░█░░░
        ░░░░█░░░░▀▀▄░█░░░█░█▀██████░█░░
        ░░░░░▀▄░░░░░▀▀▄▄▄█▄█▄█▄█▄▀░░█░░
        ░░░░░░░▀▄▄░▒▒▒▒░░░░░░░░░░▒░░░█░
        ░░░░░░░░░░▀▀▄▄░▒▒▒▒▒▒▒▒▒▒░░░░█░
        ░░░░░░░░░░░░░░▀▄▄▄▄▄░░░░░░░░█░░
         */

        intakeSim.iterate(
            intakeMotor.encoder.velocity,
            RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts.
            0.02); // Time interval, in Seconds

        jointSim.iterate(
            jointMotor.encoder.velocity,
            RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts.
            0.02); // Time interval, in Seconds

        inputs.intakeVelocityRPM = intakeMotor.getEncoder().velocity
        inputs.intakeAppliedVolts = intakeSim.busVoltage
        inputs.intakeCurrentAmps = doubleArrayOf(intakeMotor.outputCurrent)
        inputs.jointVelocityRPM = jointMotor.getEncoder().velocity
        inputs.jointAppliedVolts = intakeSim.busVoltage
        inputs.jointCurrentAmps = doubleArrayOf(jointMotor.outputCurrent)
    }
}