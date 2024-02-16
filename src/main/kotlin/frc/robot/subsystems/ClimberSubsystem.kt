package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.Command
import kotlin.math.abs

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
        val p = 0.005
        var error = positionMeters - getPositionMeters()
        while (abs(error) < 0.05){
            rightMotor.set(p*error)
            leftMotor.set(p*error)
            error = positionMeters - getPositionMeters()
        }
    }

    fun climberUpCommand():Command{
        return run{
            setToPositionMeters(1.0)
        }
    }
    
    fun climberDownCommand():Command{
        return run{
            setToPositionMeters(0.0)
        }
    }

    fun climberCentreCommand():Command{
        return run{
            setToPositionMeters(0.2)
        }
    }

    fun stop(){
        rightMotor.set(0.0)
        leftMotor.set(0.0)
    }
    
    override fun periodic() {
        
    }
    
}