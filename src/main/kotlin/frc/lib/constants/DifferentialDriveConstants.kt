package frc.lib.constants

import com.revrobotics.CANSparkMax


class DifferentialDriveConstants {
    object MotorIDs {
        val FRONT_LEFT = 0
        val REAR_LEFT = 0
        val FRONT_RIGHT = 0
        val REAR_RIGHT = 0
    }

    object MotorConsts {
        val LEFT_INVERTED = false
        val RIGHT_INVERTED = false
        val LEFT_CONVERSION_FACTOR: Double = 1.0 / 1.0
        val RIGHT_CONVERSION_FACTOR: Double = 1.0 / 1.0
        val RAMP_RATE_SECONDS: Double = 0.0
    }

    object OdometryConsts{
        val ODOMETRY_ON = false
        val KS_VOLTS:Double = 0.0
        val KV_VOLTS_SECOND_PER_METER:Double= 0.0
        val KA_VOLTS_SECONDS_PER_SQUARE_METER:Double = 0.0
        val KP_DRIVE_VELOCITY:Double = 0.0
        val TRACK_WIDTH_METERS:Double = 0.0
        val MAX_SPEED_METERS_PER_SECOND:Double = 0.0
        val MAX_ACCELERATION_METERS_PER_SECOND_SQUARED:Double = 0.0
        val K_RAMSETE_B:Double = 0.0
        val K_RAMSETE_ZETA:Double = 0.0

        val MAX_VOLTAGE:Double= 10.0
    }


}