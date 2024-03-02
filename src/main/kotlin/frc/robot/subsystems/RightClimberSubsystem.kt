package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder

class RightClimberSubsystem: LeftClimberSubsystem(){
    override val motor = CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless)

    init{
        encoder.positionConversionFactor = 1.0/1500
    }

}