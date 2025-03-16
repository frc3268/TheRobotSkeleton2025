package frc.lib.core

import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.lib.swerve.SwerveDriveBase
import frc.robot.Constants


object robotCore {
    /** The drive subsystem. Must be accessed after calling [initCore] */
    lateinit var driveSubsystem: SwerveDriveBase
    lateinit var driverController: CommandXboxController


    fun initCore() {
        // initialize HAL
        check(HAL.initialize(500, 0)) { "Failed to initialize. Terminating." }

        // report robot language as Kotlin
        // 6 Means kotlin in French
        HAL.report(FRCNetComm.tResourceType.kResourceType_Language, 6)

        driveSubsystem = SwerveDriveBase(Pose2d());
        driverController = CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT)
    }
}