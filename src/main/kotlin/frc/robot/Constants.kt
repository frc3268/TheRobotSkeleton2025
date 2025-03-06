package frc.robot

import frc.robot.Constants.Levels.LEVEL0
import frc.robot.elevator.ElevatorSubsystem

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. inside the companion object). Do not put anything functional in this class.
 *
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
object Constants {
    object OperatorConstants {
        const val DRIVER_CONTROLLER_PORT = 0
        const val STICK_DEADBAND = 0.1
    }

    /** Constants for Camera Simulation */
    object CameraSimulationConstants {
        const val CAMERA_FPS = 15.0
        const val AVERAGE_LATENCY = 50.0
        const val LATENCY_STD_DEV_MS = 15.0

        /** Should the simulated camera use wireframe? */
        const val USE_WIREFRAME = true
        /** Should the simulated camera output its raw stream to localhost:1181 */
        const val ENABLE_RAW_STREAM = true
        /** Should the simulated camera output its processed stream to localhost:1182 */
        const val ENABLE_PROCESSED_STREAM = true
    }

    /** Calibration for Camera Simulation */
    object CameraSimulationCalibration {
        const val RES_WIDTH = 960
        const val RES_HEIGHT = 720
        const val FOV_DIAGONAL = 70.0
        const val AVG_ERROR_PX = 0.35
        const val ERROR_STD_DEV_PX = 0.10
    }

    const val TROUBLESHOOTING_TAB = "Troubleshooting"

    /** How far up the [ElevatorSubsystem] should go to, as an enum corresponding to each of the reef's levels
     *
     * [LEVEL0] Should be the elevators lowest possible point / resting point
     */
    enum class Levels(val lvl: Double) {
        LEVEL0(0.0),
        LEVEL1(12.0),
        LEVEL2(24.0),
        LEVEL3(36.0),
        LEVEL4(50.0)
    }


    enum class States{REAL, SIM, REPLAY}

    /** Current state of the robot. Should be automatically set. */
    var mode = States.REAL
}
