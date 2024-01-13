package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel.MotorType
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder

/**
 * A telescoping climber.
 * @author Weiju
 */
class ClimberSubsystem : SubsystemBase() {

    /**
     * Turns clockwise to extend.
     */
    val primaryMotor = CANSparkMax(0, MotorType.kBrushless)

    /**
     * Turns counter-clockwise to extend.
     */
    val invertedMotor = CANSparkMax(0, MotorType.kBrushless)

    val primaryEncoder = primaryMotor.encoder

    /**
     * TODO Does this need to be inverted?
     */
    val invertedEncoder = invertedMotor.encoder

    /**
     * The speed at which the motors controlling the climber extends.
     * TODO Placeholder
     */
    val SPEED = 0.3

    /**
     * TODO Replace with actual conversion factor
     */
    val CLIMBER_CONVERSION_FACTOR = 360 / (147 / 1.0)

    val avgOfEncoders get() = (primaryEncoder.position + invertedEncoder.position) / 2

    init {
        primaryEncoder.setPositionConversionFactor(CLIMBER_CONVERSION_FACTOR)
        invertedEncoder.setPositionConversionFactor(CLIMBER_CONVERSION_FACTOR)
        invertedMotor.inverted = true

        primaryEncoder.position = 0.0
        invertedEncoder.position = 0.0
    }

    fun extend(): Command =
        run {
            primaryMotor.set(SPEED)
            invertedMotor.set(SPEED)
        }
        .until {
            false
        }

    fun retract(): Command =
        run {

        }
                .until {
                    false
                }

    /** This method will be called once per scheduler run  */
    override fun periodic() {
    }

    /** This method will be called once per scheduler run during simulation  */
    override fun simulationPeriodic() {
    }
}
