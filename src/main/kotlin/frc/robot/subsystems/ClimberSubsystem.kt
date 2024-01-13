package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import edu.wpi.first.wpilibj2.command.SubsystemBase

class ClimberSubsystem: SubsystemBase(){
    private val leftMotor:CANSparkMax = CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless)
    private val rightMotor:CANSparkMax = CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless)
    
    val leftEncoder:RelativeEncoder = leftMotor.encoder
    private val rightEncoder:RelativeEncoder = rightMotor.encoder
    
    init{
        rightMotor.inverted = true
        //todo:test if this is needed
        rightEncoder.inverted = true
        
        leftEncoder.positionConversionFactor = 1.0/1.0
        rightEncoder.positionConversionFactor = 1.0/1.0
        leftEncoder.position = 0.0
        rightEncoder.position = 0.0
    }
    
    fun getPositionMeters():Double = (leftEncoder.position+rightEncoder.position)/2
    
    fun setToPositionMeters(positionMeters:Double){
        //todo: replace with PID?
        while (getPositionMeters() < positionMeters){
            rightMotor.set(0.3)
            leftMotor.set(0.3)
        }
    }
    
    fun stop(){
        rightMotor.set(0.0)
        leftMotor.set(0.0)
    }
    
    override fun periodic() {
        
    }
    
}