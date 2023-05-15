package frc.lib.constants

import com.revrobotics.CANSparkMax.IdleMode
import edu.wpi.first.math.geometry.Rotation2d
import frc.lib.constants.SwerveDriveConstants.DrivetrainConsts.WHEEL_DIAMETER_METERS

class SwerveDriveConstants {
    data class ModuleConstants(
        val MODULE_NUMBER: Int,
        val ANGLE_OFFSET: Rotation2d,
        val DRIVE_MOTOR_ID: Int,
        val ANGLE_MOTOR_ID: Int,
        val ENCODER_ID: Int

    )

    object ModuleGains {
        /* Drive Motor Characterization Values */
        val DRIVE_KS: Double = 0.0
        val DRIVE_KV: Double = 0.0
        val DRIVE_KA: Double = 0.0
    }

    object DriveMotorConsts {
        val GEAR_RATIO: Double = 8.14 / 1.0
        val CONTINUOUS_CURRENT_LIMT: Int = 80
        val POSITION_CONVERSION_FACTOR_METERS_PER_ROTATION: Double =
            (WHEEL_DIAMETER_METERS * Math.PI) / GEAR_RATIO
        val VELOCITY_CONVERSION_FACTOR_METERS_PER_SECOND = POSITION_CONVERSION_FACTOR_METERS_PER_ROTATION / 60.0
        val NEUTRAL_MODE: IdleMode = IdleMode.kBrake
        val INVERT: Boolean = false


        /* Drive Motor PID Values */
        val KP: Double = 0.1
        val KI: Double = 0.0
        val KD: Double = 0.0
        val KFF: Double = 0.0
    }

    object AngleMotorConsts {
        val GEAR_RATIO: Double = 12.8 / 1.0
        val CONTINUOUS_CURRENT_LIMT: Int = 20
        val POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION = 360 / GEAR_RATIO
        val NEUTRAL_MODE: IdleMode = IdleMode.kBrake
        val INVERT: Boolean = false

        val KP: Double = 0.01
        val KI: Double = 0.0
        val KD: Double = 0.0
        val KFF: Double = 0.0

    }

    object EncoderConsts {
        val INVERT:Boolean = false
        val POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION:Double = 360/1.0
    }

    object DrivetrainConsts {        /* Drivetrain Constants */
        val TRACK_WIDTH_METERS = 21.73
        val WHEEL_BASE_METERS = 21.73

        val WHEEL_DIAMETER_METERS = 4
        val WHEEL_CIRCUMCE_METERS = WHEEL_DIAMETER_METERS * Math.PI

        val OPEN_LOOP_RAMP_RATE_SECONDS: Double = 0.25
        val CLOSED_LOOP_RAMP_RATE_SECONDS: Double = 0.0

        /* Swerve Voltage Compensation */
        val VOLTAGE_COMPENSATION: Double = 12.0

        /* Swerve Profiling Values */
        val MAX_SPEED_METERS_PER_SECOND = 4.5
        val MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND = 11.5

    }
    object modules{
        val list:List<ModuleConstants> = listOf(
            ModuleConstants(0, Rotation2d.fromDegrees(0.0), 0, 0, 0),
            ModuleConstants(0, Rotation2d.fromDegrees(0.0), 0, 0, 0),
            ModuleConstants(0, Rotation2d.fromDegrees(0.0), 0, 0, 0),
            ModuleConstants(0, Rotation2d.fromDegrees(0.0), 0, 0, 0)
        )
    }

}