package frc.lib.basics

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.cscore.CvSink
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.*
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.*
import frc.lib.constants.SwerveDriveConstants
import frc.lib.utils.*
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.photonvision.EstimatedRobotPose
import java.util.*
import kotlin.math.*

class SwerveDriveBase(startingPose: Pose2d) : SubsystemBase() {
    val field = Field2d()
    private val shuffleboardTab = Shuffleboard.getTab("Drivetrain")
    private val generalTab = Shuffleboard.getTab("General")

    var poseEstimator: SwerveDrivePoseEstimator
    private val modules: List<SwerveModule> =
        SwerveDriveConstants.modules.mapIndexed { _, swerveMod -> SwerveModule(swerveMod) }
    private val gyro = AHRS(SPI.Port.kMXP)

    private var joystickControlledEntry: GenericEntry = shuffleboardTab
            .add("Joystick Control", true)
            .withWidget("Toggle Button")
            .withPosition(2, 0)
            .withProperties(mapOf("colorWhenTrue" to "green", "colorWhenFalse" to "maroon"))
            .entry

    private var poseXEntry = shuffleboardTab.add("Pose X", 0.0)
            .withPosition(0, 0)
            .entry
    private var poseYEntry = shuffleboardTab.add("Pose Y", 0.0)
            .withPosition(1, 0)
            .entry
    private var seesAprilTag = shuffleboardTab.add("Sees April Tag?", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(2, 1)
            .entry

    private val camera:Camera

    init {
        SwerveDriveConstants.DrivetrainConsts.thetaPIDController.enableContinuousInput(
            360.0, 0.0
        )
        camera = Camera("hawkeye", "")
        zeroYaw()
        //https://github.com/Team364/BaseFalconSwerve/issues/8#issuecomment-1384799539
        Timer.delay(1.0)
        resetModulesToAbsolute()
        shuffleboardTab.add("Stop all", stopCommand())
                .withPosition(3, 0)
                .withWidget(BuiltInWidgets.kCommand)
        generalTab.add("Zero heading", zeroHeadingCommand())
                .withPosition(4, 0)
                .withWidget(BuiltInWidgets.kCommand)
        shuffleboardTab.add("Heading Angle", gyro)
                .withPosition(0, 1)
                .withWidget(BuiltInWidgets.kGyro)
        shuffleboardTab.add(field)
                .withPosition(2, 2)
                .withWidget(BuiltInWidgets.kField)

        poseEstimator = SwerveDrivePoseEstimator(SwerveDriveConstants.DrivetrainConsts.kinematics, getYaw(), getModulePositions(), startingPose, VecBuilder.fill(0.1, 0.1, 0.1),  VecBuilder.fill(0.5, 0.5, 0.5))
    }

    override fun periodic() {
        //estimate robot pose based on what the encoders say
        poseEstimator.update(getYaw(), getModulePositions())
        //estimate robot pose based on what the camera sees
        seesAprilTag.setBoolean(camera.captureFrame().hasTargets())
        /*
        camera.getEstimatedPose()?.ifPresent { est ->
            poseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds
            , camera.getEstimationStdDevs(est.estimatedPose.toPose2d()))
        }
         */
        //update module tabs on shuffleboard
        for (mod in modules){
            mod.updateShuffleboard()
        }
        //update drivetrain tab on shuffleboard
        field.robotPose = getPose()
        poseXEntry.setDouble(getPose().x)
        poseYEntry.setDouble(getPose().y)
    }

    override fun simulationPeriodic() {
        for ((x, state) in constructModuleStatesFromChassisSpeeds(0.0, 0.0,0.1, true).withIndex()){
            modules[x].setPointEntry.setDouble(state.angle.degrees)
        }
    }

    fun zeroYaw() {
        gyro.reset()
    }

    private fun resetModulesToAbsolute() {
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
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSecond,ySpeedMetersPerSecond,turningSpeedDegreesPerSecond.rotation2dFromDeg().radians,getPose().rotation)
            else
                ChassisSpeeds(xSpeedMetersPerSecond,ySpeedMetersPerSecond,turningSpeedDegreesPerSecond.rotation2dFromDeg().radians)
        )
    fun stopAll(){
        for (mod in modules) {
            mod.stop()
        }
    }

    fun stopCommand() : Command =
        run { stopAll() }
            .until { joystickControlledEntry.getBoolean(true) }
            .beforeStarting (runOnce { joystickControlledEntry.setBoolean(false) } )

    //reset yaw on gyro so that wherever the gyro is pointing is the new forward(0) value
    fun zeroHeadingCommand(): Command =
        runOnce { zeroYaw() }

    //move robot to pose given in endpose argument
    fun moveToPoseCommand(endPose: Pose2d): Command =
        run {
            setModuleStates(
                constructModuleStatesFromChassisSpeeds(
                -SwerveDriveConstants.DrivetrainConsts.xPIDController.calculate(getPose().x,  endPose.x),
                -SwerveDriveConstants.DrivetrainConsts.yPIDController.calculate(getPose().y,  endPose.y),
                SwerveDriveConstants.DrivetrainConsts.thetaPIDController.calculate(getPose().rotation.degrees,  endPose.rotation.degrees),
                true
            ))
            println(getPose().translation.getDistance(endPose.translation))
        }.until {
            (abs(getPose().translation.getDistance(endPose.translation)) < 0.25
                && abs(getPose().rotation.degrees - endPose.rotation.degrees) < 15) || joystickControlledEntry.getBoolean(true)
        }.andThen(stopCommand()) .beforeStarting (runOnce { joystickControlledEntry.setBoolean(false) } )

    //getters
    fun getYaw(): Rotation2d = -(gyro.rotation2d.degrees).IEEErem(360.0).rotation2dFromDeg()
    fun getPitch(): Rotation2d = gyro.pitch.toDouble().rotation2dFromDeg()
    fun getPose():Pose2d = Pose2d(poseEstimator.estimatedPosition.x, poseEstimator.estimatedPosition.y, poseEstimator.estimatedPosition.rotation)
    fun getModuleStates(): Array<SwerveModuleState> = modules.map { it.getState() }.toTypedArray()
    fun getModulePositions(): Array<SwerveModulePosition> = modules.map { it.getPosition() }.toTypedArray()

    //sets poseEstimator's recorded position to a pose2d given in startingpose argument
    fun zeroPoseToFieldPosition(startingPose: Pose2d){
        poseEstimator.resetPosition(getYaw(), getModulePositions(), startingPose)
    }

    //does the same thing as previous but with the camera finding the starting pose
    fun zeroPoseToCameraPosition(){
        val visionEst: Optional<EstimatedRobotPose>? = camera.getEstimatedPose()
        visionEst?.ifPresent { est ->
            zeroPoseToFieldPosition(est.estimatedPose.toPose2d())
        }
    }

    fun getPoseOfNote(cvSink: CvSink, matrix: Mat, hierarchy: Mat) : Pose2d{
        if(cvSink.grabFrame(matrix).toInt() == 0){
            //if there's an error, the robot won't move
            return getPose()
        }
        //35 100 100
        //5 80 80
        Imgproc.cvtColor(matrix, matrix, Imgproc.COLOR_RGB2HSV)
        Core.inRange(matrix, Scalar(5.0, 80.0, 80.0), Scalar(35.0, 100.0, 100.0), matrix)
        var contours:List<MatOfPoint> = listOf()
        //there may be an issue with this?
        Imgproc.findContours(matrix, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)
        for(contour: MatOfPoint in contours){
            val moments = Imgproc.moments(contour)
            if(moments._m00.toInt() != 0){
                val cx = (moments._m10/moments._m00).toInt()
                val cy = (moments._m01/moments._m00).toInt()
                val area = moments._m00

                println("cx, cy:$cx,$cy")
                println("area: $area")
            }

        }

        return getPose()
    }

}
