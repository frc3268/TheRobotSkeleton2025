package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Translation2d
import frc.lib.basics.SwerveDriveBase
import frc.lib.constants.SwerveDriveConstants
import java.util.function.DoubleSupplier
import java.util.function.BooleanSupplier

import frc.robot.Constants

class SwerveJoystickDrive(
    drive: SwerveDriveBase,
    translationX: DoubleSupplier,
    translationY: DoubleSupplier,
    rotation: DoubleSupplier,
    fieldOriented: BooleanSupplier
) : CommandBase() {
    val drive: SwerveDriveBase = drive
    val translationX: DoubleSupplier = translationX
    val translationY: DoubleSupplier = translationY
    val rotation: DoubleSupplier = rotation
    val fieldOriented: BooleanSupplier = fieldOriented

    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive)
    }

    // Called when the command is initially scheduled.
    override fun initialize() { }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() { 
        /* Get Values, Deadband, Convert to speeds */
        val xSpeed: Double = MathUtil.applyDeadband(translationX.asDouble, Constants.OperatorConstants.STICK_DEADBAND)* SwerveDriveConstants.DrivetrainConsts.MAX_SPEED_METERS_PER_SECOND
        val ySpeed: Double = MathUtil.applyDeadband(translationY.asDouble, Constants.OperatorConstants.STICK_DEADBAND) * SwerveDriveConstants.DrivetrainConsts.MAX_SPEED_METERS_PER_SECOND
        val turnSpeed: Double = MathUtil.applyDeadband(rotation.asDouble, Constants.OperatorConstants.STICK_DEADBAND) * SwerveDriveConstants.DrivetrainConsts.MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND

        /* Drive */
        drive.setModuleStates(drive.constructStates(xSpeed,ySpeed,turnSpeed,fieldOriented.asBoolean))
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
        drive.stop()
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
