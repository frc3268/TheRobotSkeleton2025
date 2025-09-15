/*
package frc.lib.basics

import com.kauailabs.navx.frc.AHRS
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.RamseteController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.trajectory.TrajectoryConfig
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.lib.constants.DifferentialDriveConstants

open class DifferentialDriveBase(startingPose: Pose2d?) : SubsystemBase() {
    //motors-required
    private val frontLeftMotor = CANSparkMax(DifferentialDriveConstants.MotorIDs.FRONT_LEFT, CANSparkLowLevel.MotorType.kBrushless)
    private val rearLeftMotor = CANSparkMax(DifferentialDriveConstants.MotorIDs.REAR_LEFT, CANSparkLowLevel.MotorType.kBrushless)
    private val frontRightMotor = CANSparkMax(DifferentialDriveConstants.MotorIDs.FRONT_RIGHT, CANSparkLowLevel.MotorType.kBrushless)
    private val rearRightMotor = CANSparkMax(DifferentialDriveConstants.MotorIDs.REAR_RIGHT, CANSparkLowLevel.MotorType.kBrushless)

    //groups-motors
    private val leftMotors = MotorControllerGroup(frontLeftMotor, rearLeftMotor)
    private val rightMotors = MotorControllerGroup(frontRightMotor, rearRightMotor)

    //Drive
    private val drive = DifferentialDrive(leftMotors, rightMotors)

    //encoders-motors
    private val frontLeftEncoder: RelativeEncoder = frontLeftMotor.encoder
    private val rearLeftEncoder: RelativeEncoder = rearLeftMotor.encoder
    private val frontRightEncoder: RelativeEncoder = frontRightMotor.encoder
    private val rearRightEncoder: RelativeEncoder = rearRightMotor.encoder

    //gyro-optional
    private var gyro: AHRS? = null

    //kinematics-optional
    private val kinematics: DifferentialDriveKinematics = DifferentialDriveKinematics(DifferentialDriveConstants.OdometryConsts.TRACK_WIDTH_METERS)

    //odometry-optional
    private var poseEstimator: DifferentialDrivePoseEstimator? = null

    //ramsete command ingredients
    val feedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(
        DifferentialDriveConstants.OdometryConsts.KS_VOLTS,
        DifferentialDriveConstants.OdometryConsts.KV_VOLTS_SECOND_PER_METER,
        DifferentialDriveConstants.OdometryConsts.KA_VOLTS_SECONDS_PER_SQUARE_METER
    )
    val voltageConstraint: DifferentialDriveVoltageConstraint = DifferentialDriveVoltageConstraint(
        feedforward,
        kinematics,
        DifferentialDriveConstants.OdometryConsts.MAX_VOLTAGE
    )
    val trajectoryConfig: TrajectoryConfig = TrajectoryConfig(
        DifferentialDriveConstants.OdometryConsts.MAX_SPEED_METERS_PER_SECOND,
        DifferentialDriveConstants.OdometryConsts.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
    )
    val ramseteController: RamseteController = RamseteController(DifferentialDriveConstants.OdometryConsts.K_RAMSETE_B, DifferentialDriveConstants.OdometryConsts.K_RAMSETE_ZETA)
    val leftMotorsPIDController: PIDController = PIDController(DifferentialDriveConstants.OdometryConsts.KP_DRIVE_VELOCITY, 0.0, 0.0)
    val rightMotorsPIDController: PIDController = PIDController(DifferentialDriveConstants.OdometryConsts.KP_DRIVE_VELOCITY, 0.0, 0.0)
    //camera-optional

    init {
        //set up encoders
        frontLeftEncoder.inverted = DifferentialDriveConstants.MotorConsts.LEFT_INVERTED
        rearLeftEncoder.inverted = DifferentialDriveConstants.MotorConsts.LEFT_INVERTED
        frontRightEncoder.inverted = DifferentialDriveConstants.MotorConsts.RIGHT_INVERTED
        rearRightEncoder.inverted = DifferentialDriveConstants.MotorConsts.RIGHT_INVERTED

        frontLeftEncoder.positionConversionFactor = DifferentialDriveConstants.MotorConsts.LEFT_CONVERSION_FACTOR
        rearLeftEncoder.positionConversionFactor = DifferentialDriveConstants.MotorConsts.LEFT_CONVERSION_FACTOR
        frontRightEncoder.positionConversionFactor = DifferentialDriveConstants.MotorConsts.RIGHT_CONVERSION_FACTOR
        rearRightEncoder.positionConversionFactor = DifferentialDriveConstants.MotorConsts.RIGHT_CONVERSION_FACTOR

        frontLeftEncoder.velocityConversionFactor = 60 / DifferentialDriveConstants.MotorConsts.LEFT_CONVERSION_FACTOR
        rearLeftEncoder.velocityConversionFactor = 60 / DifferentialDriveConstants.MotorConsts.LEFT_CONVERSION_FACTOR
        frontRightEncoder.velocityConversionFactor = 60 / DifferentialDriveConstants.MotorConsts.RIGHT_CONVERSION_FACTOR
        rearRightEncoder.velocityConversionFactor = 60 / DifferentialDriveConstants.MotorConsts.RIGHT_CONVERSION_FACTOR

        frontLeftMotor.setOpenLoopRampRate(DifferentialDriveConstants.MotorConsts.RAMP_RATE_SECONDS)
        rearLeftMotor.setOpenLoopRampRate(DifferentialDriveConstants.MotorConsts.RAMP_RATE_SECONDS)
        frontRightMotor.setOpenLoopRampRate(DifferentialDriveConstants.MotorConsts.RAMP_RATE_SECONDS)
        frontRightMotor.setOpenLoopRampRate(DifferentialDriveConstants.MotorConsts.RAMP_RATE_SECONDS)

        resetEncoders()
        //odometry: check if it's real here and initialize
        if (DifferentialDriveConstants.OdometryConsts.ODOMETRY_ON) {
            try {
                gyro = AHRS(SPI.Port.kMXP)
                poseEstimator = DifferentialDrivePoseEstimator(
                    kinematics,
                    getYaw(),
                    getLeftEncodersDistance(),
                    getRightEncodersDistance(),
                    startingPose
                )
            } catch (ex: RuntimeException) {
                DriverStation.reportError("Odometry failed to start. Ensure that ODOMETRY_ON in DifferentialDriveConstants.kt is set to false if you do not plan to use odometry", false)
            }

        }
    }

    override fun periodic() {
        //update odometry if the object exists
        poseEstimator?.update(getYaw(), getLeftEncodersDistance(), getRightEncodersDistance())
        poseEstimator?.let { gyro?.let { it1 -> updateOdometry(it, it1) } }
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    //--ENCODERS--//
    fun getLeftEncodersDistance(): Double = (frontLeftEncoder.position + rearLeftEncoder.position) / 2
    fun getRightEncodersDistance(): Double = (frontRightEncoder.position + rearRightEncoder.position) / 2
    fun getEncodersDistance(): Double = (getLeftEncodersDistance() + getRightEncodersDistance()) / 2

    fun getLeftEncodersVelocity(): Double = (frontLeftEncoder.velocity + rearLeftEncoder.velocity) / 2
    fun getRightEncodersVelocity(): Double = (frontRightEncoder.velocity + rearRightEncoder.velocity) / 2
    fun getEncodersVelocity(): DifferentialDriveWheelSpeeds = DifferentialDriveWheelSpeeds(getLeftEncodersVelocity(), getRightEncodersVelocity())

    fun resetEncoders() {
        frontLeftEncoder.position = 0.0
        rearLeftEncoder.position = 0.0
        frontRightEncoder.position = 0.0
        rearRightEncoder.position = 0.0
    }

    //--ODOM--//
    //override for custom odom code
    open fun updateOdometry(odometry: DifferentialDrivePoseEstimator, gyro: AHRS) {
        return
    }

    fun resetOdometry(pose: Pose2d) {
        resetEncoders()
        poseEstimator?.resetPosition(
            getYaw(), getLeftEncodersDistance(), getRightEncodersDistance(), pose)
    }

    fun getPose(): Pose2d? {
        return poseEstimator?.estimatedPosition
    }

    fun getYaw(): Rotation2d? {
        return gyro?.rotation2d
    }

    fun getPitch(): Rotation2d? {
        return gyro?.pitch?.toDouble()?.let { Rotation2d.fromDegrees(it) }
    }

    //--DRIVE--//
    fun arcadeDrive(forward: Double, rotation: Double) {
        drive.arcadeDrive(forward, rotation)
    }

    fun tankDriveVolts(leftVolts: Double, rightVolts: Double) {
        leftMotors.setVoltage(leftVolts)
        rightMotors.setVoltage(rightVolts)
        drive.feed()
    }

}
*/