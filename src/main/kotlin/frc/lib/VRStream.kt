package frc.lib

import org.opencv.core.*
import org.opencv.videoio.VideoCapture
import edu.wpi.first.wpilibj.DriverStation

class VRStream {

    private var loaded: Boolean = false;

    var fps: UInt = 30u;

    init {
        // TODO: Load Webcam
        val video = VideoCapture(0)
        if (video.isOpened) {
            // Do Something I guess
            this.loaded = true;
        }
        else {
            //DriverStation.reportError("VRStream: Failed to Load")
        }

    }
}