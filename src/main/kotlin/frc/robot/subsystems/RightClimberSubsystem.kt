package frc.robot.subsystems

import com.revrobotics.*
import edu.wpi.first.wpilibj2.command.*

class RightClimberSubsystem: SubsystemBase(){
    val motor = CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless)
    val encoder: RelativeEncoder = motor.encoder

    /* CONSTANTS */
    private val metersPerRotation: Double = 0.0
    private val minPositionMeters: Double = 0.0
    private val maxPositionMeters: Double = 0.34

    init {
        motor.inverted = true
        // TODO test if this is needed
        //rightEncoder.inverted = true

        encoder.positionConversionFactor = 1.0/1500
        encoder.position = 0.0
    }

    /**
     * TODO make a func that converts meters to rotations and a function that converts rotations to meters
     * TODO using these functions rotate the motors on the arms accordingly while accounting for min and max heights
     */

    fun down(): Command =
        run { motor.set(-0.7) }
            .until { encoder.position < 0.1 }
            .andThen(runOnce { motor.stopMotor() })

    fun up(): Command =
        run { motor.set(0.7) }
            .until { encoder.position > 0.9 }
            .andThen(runOnce { motor.stopMotor() })

    fun reset(): Command =
        runOnce { encoder.position = 0.0 }

    fun climberCentreCommand(): Command =
        run { //setToPositionMeters(0.2)
        }

    fun stop(): Command =
        runOnce { motor.set(0.0) }

    override fun periodic() {
        System.out.println("Left climber: " + encoder.position)
    }
}