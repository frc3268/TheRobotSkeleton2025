package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.Command

class LeftClimberSubsystem: SubsystemBase(){
    private val motor = CANSparkMax(14, CANSparkLowLevel.MotorType.kBrushless)

    private val encoder: RelativeEncoder = motor.encoder

    /* CONSTANTS */

    private val metersPerRotation: Double = 0.0
    private val minPositionMeters: Double = 0.0
    private val maxPositionMeters: Double = 0.34
    
    init {
        motor.inverted = true
        // TODO test if this is needed
        //rightEncoder.inverted = true
        
        encoder.positionConversionFactor = 1.0/530
        encoder.position = 0.0
    }
    /**
     * TODO make a func that converts meters to rotations and a function that converts rotations to meters
     * TODO using these functions rotate the motors on the arms accordingly while accounting for min and max heights
     */

    fun climberUpCommand(): Command =
        run {
            //setToPositionMeters(1.0)
        }
    
    fun climberDownCommand():Command =
        run {
            //setToPositionMeters(0.0)
        }


    fun down() : Command =
            run{
                motor.set(-0.7)
            }.until  {encoder.position < 0.1}.andThen(runOnce{motor.stopMotor()})

    fun up() : Command =
            run{
                motor.set(0.7)
            }.until  {encoder.position > 0.9}.andThen(runOnce{motor.stopMotor()})



    fun reset(): Command{
        return runOnce{
            encoder.position = 0.0
        }
    }


    fun climberCentreCommand():Command =
            run {
                //setToPositionMeters(0.2)
            }

    fun stop():Command{
        return runOnce {
            motor.set(0.0)
        }
    }

    override fun periodic() {

        System.out.println(encoder.position)
    }


}