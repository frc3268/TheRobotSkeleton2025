package frc.lib.utils.pidtuner

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMax.ControlType
import edu.wpi.first.wpilibj2.command.SubsystemBase

//may add feedforward later
data class PidConstants(var p:Double = 0.0, var i:Double = 0.0, var d:Double = 0.0)

class PidTunerSubsystem(startingValues: PidConstants?, private val motor: CANSparkMax): SubsystemBase() {
    //TODO:ui on smartdashboard/shuffleboard(?)
    //TODO:calibrate function(1 for each mechanism type)

    var coefficients = startingValues ?: PidConstants()
    val encoder = motor.encoder
    val controller = motor.pidController
    val latestReadings:MutableList<Double> = mutableListOf()
    override fun periodic() {

    }

    fun setReference(goal_rotations:Double){
        controller.setReference(goal_rotations, ControlType.kPosition);
    }

    fun updateCoefficients(coefficients: PidConstants){
        controller.p = coefficients.p
        controller.i = coefficients.i
        controller.d = coefficients.d
    }
    fun set(speed: Double) {
        motor.set(speed)
    }

    fun getPositionRotations(): Double {
        return encoder.position
    }

    fun getPowerSent() : Double{
        return motor.appliedOutput
    }

    fun resetEncoderAtPos(){
        encoder.setPosition(0.0)
    }

    fun resetReadings(){
        latestReadings.clear()
    }

    fun addReading(reading: Double){
        latestReadings.add(reading)
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    //uses algorithm similar to what is detailed by https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html
    fun positiontune(goalDistanceRotations:Double, tuningAmplitude: Double) : PidConstants {
        //first, lets find the greatest value read
        val max = latestReadings.maxOrNull() ?: return coefficients
        val min = latestReadings.minOrNull() ?: return coefficients
        if (goalDistanceRotations <= 0 || max <= 0 || min <= 0) {
            //cant do much if goal, max, or min aren't above 0
            return coefficients
        }
        val tuned = coefficients
        if(max < goalDistanceRotations){
            //if we don't even reach the goal, it can be assumed that I is not high enough
            tuned.i += ((goalDistanceRotations - max) / goalDistanceRotations) * tuningAmplitude
            //no need to modify anything else yet
        }
        //next, let's look for oscillation
        if(max > latestReadings.last()){
            //if we went forward then back, we need to increase the D term
            tuned.d += ((max - latestReadings.last()) / max) * tuningAmplitude
            //count oscillations
            var oscillations = 0
            var gonePast = false
            for (reading in latestReadings){
                if(reading > goalDistanceRotations) {
                    gonePast = true
                }
                if(gonePast && reading < goalDistanceRotations){
                    oscillations += 1
                    gonePast = false
                }
            }
            tuned.p -= tuningAmplitude * (oscillations + 1)
        } else{
            tuned.p += tuningAmplitude
        }

        return tuned
    }

}