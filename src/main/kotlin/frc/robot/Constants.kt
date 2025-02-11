package frc.robot

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
        const val kDriverControllerPort = 0
        const val STICK_DEADBAND = 0.1
    }

    object SimulationConstants {
        const val CAMERA_FPS = 15.0;
        const val AVERAGE_LATENCY = 50.0;
        const val LATENCY_STD_DEV_MS = 15.0;
        const val USE_WIREFRAME = true;
        const val ENABLE_RAW_STREAM = true;
        const val ENABLE_PROCESSED_STREAM = true;
    }

    object SimulationCalibration {
        const val RES_WIDTH = 960
        const val RES_HEIGHT = 720
        const val FOV_DIAG = 90.0
        const val AVG_ERROR_PX = 0.35;
        const val ERROR_STD_DEV_PX = 0.10;
    }

    const val TROUBLESHOOTING_TAB = "Troubleshooting"

    const val ELE_POS_1 = 0;
    const val ELE_POS_2 = 0;
    const val ELE_POS_3 = 0;
    const val ELE_POS_4 = 0;

    enum class States{REAL, SIM, REPLAY}

    //any way to make this not hardcoded?
    var mode = States.SIM
}
