# Swerve Subsystem
**Prerequisite Knowledge:**
- Introduction

**As mentioned in the links on the Intro page, every mechanism on a robot is its own subsystem.** That includes the chassis/drivebase. The major components of a Swerve Chassis are as follows:
- Swerve Modules(x4): Control and track the movement of the wheels on the chassis
- Gyroscope: Senses the pitch, roll, and yaw of the chassis.

In essence, code for a Swerve Chassis needs to do the following:
- Control the movement of the chassis, provide interfaces for such(so that other parts of the code can control the movement of the chassis)
- Report the position and state of the Chassis
- Interface with the Gyroscope

The code for the Swerve Chassis is stored in the `src/main/kotlin/frc/lib/basics/SwerveDriveBase.kt` file of the ValhallaLib repo. 

_But what does each part do?_

**Lines 25-56 (Setup):**
```kt
class SwerveDriveBase(startingPose: Pose2d) : SubsystemBase() {
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


    var estimatedheadingentry:GenericEntry = ShuffleboardTab.add("Estimated Robot Heading", 0.0).withWidget(BuiltInWidgets.kGyro).entry

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
        //todo: a button to restart driving
        ShuffleboardTab.add("Robot Heading", gyro).withWidget(BuiltInWidgets.kGyro)


    }
```
This section is similar to the first section of the `SwerveModule` class, as it involves declaration of the `SwerveDriveBase` class and the variables within. The `SwerveDriveBase` class extends the `SubsystemBase` class provided by WPILIB, due to the reasons discussed at the start of this document.
