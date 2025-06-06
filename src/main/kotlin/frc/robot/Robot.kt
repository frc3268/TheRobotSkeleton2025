package frc.robot

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup
import edu.wpi.first.wpilibj.motorcontrol.Talon
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
class Robot : TimedRobot() {
    private var autonomousCommand: Command? = null

    val driveleftFront = Talon(0);
    val driveleftBack = Talon(1);
    val driverightFront = Talon(2);
    val driverightBack = Talon(3);
    var basepower: Double = 0.0

    val PD = PowerDistribution(0, PowerDistribution.ModuleType.kCTRE);
    val driverController = CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT)

    var drive: DifferentialDrive = DifferentialDrive(
        MotorControllerGroup(driveleftFront, driveleftBack),
        MotorControllerGroup(driverightFront, driverightBack))

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    override fun robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        if (isReal()) {
            Constants.mode = Constants.States.REAL
            PowerDistribution(1, PowerDistribution.ModuleType.kRev) // Enables power distribution logging
        } else if (isSimulation()) {
            Constants.mode = Constants.States.SIM
            // Running a physics simulator, log to NT
            Logger.addDataReceiver(NT4Publisher())
        } else {
            Constants.mode = Constants.States.REPLAY
            val logPath = LogFileUtil.findReplayLog() // Pull the replay log from AdvantageScope (or prompt the user)
            Logger.setReplaySource(WPILOGReader(logPath)) // Read replay log
            Logger.addDataReceiver(
                WPILOGWriter(
                    LogFileUtil.addPathSuffix(
                        logPath, "_sim"
                    )
                )
            ) // Save outputs to a new log
        }

    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     *
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    override fun robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run()
    }

    /** This function is called once each time the robot enters Disabled mode.  */
    override fun disabledInit() {}

    /** This function is called periodically when disabled.  */
    override fun disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your [RobotContainer] class.  */
    override fun autonomousInit() {
//        autonomousCommand = robotContainer?.autonomousCommand
//        // Schedule the autonomous command (example)
//        // Note the Kotlin safe-call(?.), this ensures autonomousCommand is not null before scheduling it
//        autonomousCommand?.schedule()
    }

    /** This function is called periodically during autonomous.  */
    override fun autonomousPeriodic() {}

    /** This function is called once when teleop is enabled.  */
    override fun teleopInit() {
        basepower = PD.voltage;
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        // Note the Kotlin safe-call(?.), this ensures autonomousCommand is not null before cancelling it
        autonomousCommand?.cancel()
        //robotContainer?.driveSubsystem?.zeroHeadingCommand()?.schedule()
        //robotContainer?.teleopCommand?.schedule()
    }

    /** This function is called periodically during operator control.  */
    override fun teleopPeriodic() {
        if(PD.voltage < 0.75 * basepower){
            //brownout prevention
            drive.arcadeDrive(driverController.rightX*0.5, driverController.leftY*0.5);
        }
        else{
            drive.arcadeDrive(driverController.rightX*0.5, driverController.leftY*0.5);
        }

    }

    /** This function is called once when test mode is enabled.  */
    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
    }

    /** This function is called periodically during test mode.  */
    override fun testPeriodic() {}

    /** This function is called once when the robot is first started up.  */
    override fun simulationInit() {}

    /** This function is called periodically whilst in simulation.  */
    override fun simulationPeriodic() {}
}
