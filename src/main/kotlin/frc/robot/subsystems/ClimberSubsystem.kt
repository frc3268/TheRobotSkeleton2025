package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.Command
import kotlin.math.abs

class ClimberSubsystem: SubsystemBase(){
    private val leftMotor = CANSparkMax(14, CANSparkLowLevel.MotorType.kBrushless)
    private val rightMotor = CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless)
    
    private val leftEncoder: RelativeEncoder = leftMotor.encoder
    private val rightEncoder: RelativeEncoder = rightMotor.encoder

    /* CONSTANTS */

    // TODO determine conversion
    private val metersPerRotation: Double = 0.0
    private val minPositionMeters: Double = 0.0
    // TODO determine max position
    // TODO 34 cm real max
    private val maxPositionMeters: Double = 0.34
    
    init {
        rightMotor.inverted = true
        // TODO test if this is needed
        rightEncoder.inverted = true
        
        leftEncoder.positionConversionFactor = 1.0/1.0
        rightEncoder.positionConversionFactor = 1.0/1.0
        leftEncoder.position = 0.0
        rightEncoder.position = 0.0
    }
    /**
     * TODO make a func that converts meters to rotations and a function that converts rotations to meters
     * TODO using these functions rotate the motors on the arms accordingly while accounting for min and max heights
     */
    fun getRotations(): Double = (leftEncoder.position + rightEncoder.position)/2

    fun getPositionMeters(): Double = getRotations() * metersPerRotation

    fun setPositionMeters(positionMeters: Double){
        // Restrict actualPositionMeters to within the bounds
        val actualPositionMeters =
                if (positionMeters < minPositionMeters) minPositionMeters
                else if (positionMeters > maxPositionMeters) maxPositionMeters
                else positionMeters
        val proportion = 0.005
        var error = actualPositionMeters / metersPerRotation - getPositionMeters()

        while (abs(error) < 0.05){
            rightMotor.set(proportion*error)
            leftMotor.set(proportion*error)
            error = actualPositionMeters - getPositionMeters()
        }
    }

    fun climberUpCommand():Command{
        return run{
            //setToPositionMeters(1.0)
        }
    }
    
    fun climberDownCommand():Command{
        return run{
            //setToPositionMeters(0.0)
        }
    }

    fun climberCentreCommand():Command{
        return run{
            //setToPositionMeters(0.2)
        }
    }

    fun stop(){
        rightMotor.set(0.0)
        leftMotor.set(0.0)
    }
    
    override fun periodic() {
        
    }
    
}