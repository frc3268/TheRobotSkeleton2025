package frc.lib.basics

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.lib.constants.SwerveDriveConstants
import frc.lib.utils.rotation2dFromDeg
import frc.lib.utils.scopeAngle

class SwerveDriveBase(startingPose: Pose2d) : SubsystemBase() {
    val field:Field2d = Field2d()
    private val ShuffleboardTab = Shuffleboard.getTab("Drivetrain")

    val poseEstimator: SwerveDrivePoseEstimator
    private val modules: List<SwerveModule> =
        SwerveDriveConstants.modules.list.mapIndexed { _, swerveMod -> SwerveModule(swerveMod) }
    private val gyro: AHRS = AHRS(SPI.Port.kMXP)

    private var joystickControlledEntry: GenericEntry =  ShuffleboardTab
        .add("Joystick Control", true)
        .withWidget("Toggle Button")
        .withProperties(mapOf("colorWhenTrue" to "green", "colorWhenFalse" to "maroon"))
        .getEntry()


    init {
        gyro.calibrate()
        zeroYaw()
        poseEstimator = SwerveDrivePoseEstimator(SwerveDriveConstants.DrivetrainConsts.kinematics, getYaw(), getModulePositions(), startingPose)
        //https://github.com/Team364/BaseFalconSwerve/issues/8#issuecomment-1384799539
        Timer.delay(1.0)
        resetModulesToAbsolute()
        ShuffleboardTab.add("Stop", stopCommand()).withWidget(BuiltInWidgets.kCommand)
        ShuffleboardTab.add("Zero Heading", zeroHeadingCommand()).withWidget(BuiltInWidgets.kCommand)
        ShuffleboardTab.add("Dig In", digInCommand()).withWidget(BuiltInWidgets.kCommand)
        ShuffleboardTab.add("Robot Heading", gyro).withWidget(BuiltInWidgets.kGyro)

        //pending review: should the field be on the drivetrain's panel or somewhere else?
        //todo: consult with drive(chris) about this
        ShuffleboardTab.add(field).withWidget(BuiltInWidgets.kField)


    }


    override fun periodic() {
        poseEstimator.update(getYaw(), getModulePositions())
        for (mod in modules){
            mod.updateShuffleboard()
        }
        //matthew try to read challenge - why does this say "traj"????
        field.getObject("traj").pose = poseEstimator.estimatedPosition

    }

    override fun simulationPeriodic() {
        for ((x, state) in constructModuleStatesFromChassisSpeeds(0.0, 0.0,0.1, true).withIndex()){
            modules[x].setPointEntry.setDouble(state.angle.degrees)
        }
    }


    private fun zeroYaw() {
        gyro.zeroYaw()
    }
    private fun resetModulesToAbsolute(){
        for (mod in modules){
            mod.resetToAbsolute()
        }
    }

    fun setModuleStates(desiredStates: Array<SwerveModuleState>) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveDriveConstants.DrivetrainConsts.MAX_SPEED_METERS_PER_SECOND)

        for (mod in modules) {
            mod.setDesiredState(desiredStates[mod.moduleConstants.MODULE_NUMBER - 1])
        }
    }

    fun constructModuleStatesFromChassisSpeeds(xSpeedMetersPerSecond:Double, ySpeedMetersPerSecond:Double, turningSpeedDegreesPerSecond:Double, fieldOriented:Boolean) : Array<SwerveModuleState> =
        SwerveDriveConstants.DrivetrainConsts.kinematics.toSwerveModuleStates (
            if (fieldOriented)
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSecond,ySpeedMetersPerSecond,turningSpeedDegreesPerSecond.rotation2dFromDeg().radians,getYaw())
            else
                ChassisSpeeds(xSpeedMetersPerSecond,ySpeedMetersPerSecond,turningSpeedDegreesPerSecond.rotation2dFromDeg().radians)
        )
    fun stop(){
        for(mod in modules){
            mod.stop()
        }
    }

    fun stopCommand() :CommandBase{
        return run {stop()}.until { joystickControlledEntry.getBoolean(true) }.beforeStarting (runOnce{joystickControlledEntry.setBoolean(false)} )
    }


     fun digInCommand(): CommandBase{
         //todo: there might be a better way to do this. Im hesitant to use a global, though.
       return runOnce{
            joystickControlledEntry.setBoolean(false)
            //because driving can be field-oriented, we need to take an angle perprendicular to the direction of the wheels, so that motion stops if we are being pushed
            }.andThen(
            run{
                setModuleStates(
                    arrayOf(SwerveModuleState(0.01, 45.0.rotation2dFromDeg()),
                            SwerveModuleState(0.01, (-45.0).rotation2dFromDeg()),
                            SwerveModuleState(0.01, 45.0.rotation2dFromDeg()),
                            SwerveModuleState(0.01, (-45.0).rotation2dFromDeg())
                ))
            }.until {joystickControlledEntry.getBoolean(true)})
    }

    fun zeroHeadingCommand(): CommandBase {
        // Inline construction of command goes here.
        // runOnce implicitly requires this subsystem.
        return runOnce { zeroYaw() }
    }
    fun getYaw(): Rotation2d = scopeAngle((gyro.rotation2d.degrees).rotation2dFromDeg())
    fun getPitch(): Rotation2d = gyro.pitch.toDouble().rotation2dFromDeg()
    fun getPose():Pose2d = poseEstimator.estimatedPosition
    fun getModuleStates(): Array<SwerveModuleState> = modules.map { it.getState() }.toTypedArray()
    fun getModulePositions(): Array<SwerveModulePosition> = modules.map { it.getPosition() }.toTypedArray()

}
