package frc.lib

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.*
import edu.wpi.first.math.geometry.*
import frc.robot.Constants
import edu.wpi.first.math.numbers.*
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import org.photonvision.*
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim
import org.photonvision.targeting.*
import java.io.IOException
import java.util.*

class Camera(name: String) {
    private val limelight = PhotonCamera(name)
    var frame = PhotonPipelineResult()
    private var poseEstimator: PhotonPoseEstimator? = null

    private var visionSim: VisionSystemSim? = null;

    private var robotToCam = Transform3d(
        Translation3d(
            Units.inchesToMeters(0.0),
            Units.inchesToMeters(12.0),
            Units.inchesToMeters(15.0),
        ),
        Rotation3d(
            0.0,
            45.0,
            0.0
        )
    )

    //private var cameraProp: SimCameraProperties? = null
    //private var visionSim: VisionSystemSim?: = null

    init {
        try {

            poseEstimator =
                    PhotonPoseEstimator(
                        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
                            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                            robotToCam
                    )
        } catch (e: IOException) {
            DriverStation.reportError("AprilTag: Failed to Load", e.stackTrace)
            // !add some way to lock down apriltage features after this
        }

        if (Constants.mode == Constants.States.SIM) {
            val cameraProp = SimCameraProperties();
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = VisionSystemSim(name);
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim?.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField));
            // Create simulated camera properties. These can be set to mimic your actual camera.
            // TODO: Configure the camera!
            cameraProp.setCalibration(
                Constants.SimulationCalibration.RES_WIDTH,
                Constants.SimulationCalibration.RES_HEIGHT,
                Rotation2d.fromDegrees(Constants.SimulationCalibration.FOV_DIAG)
            );
            cameraProp.setCalibError(
                Constants.SimulationCalibration.AVG_ERROR_PX,
                Constants.SimulationCalibration.ERROR_STD_DEV_PX
            );

            cameraProp.setFPS(Constants.SimulationConstants.CAMERA_FPS);
            cameraProp.setAvgLatencyMs(Constants.SimulationConstants.AVERAGE_LATENCY);
            cameraProp.setLatencyStdDevMs(Constants.SimulationConstants.LATENCY_STD_DEV_MS);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            var cameraSim = PhotonCameraSim(limelight, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim?.addCamera(cameraSim, robotToCam);

            // Enable the raw and processed streams. These are enabled by default.
            cameraSim.enableRawStream(Constants.SimulationConstants.ENABLE_RAW_STREAM);
            cameraSim.enableProcessedStream(Constants.SimulationConstants.ENABLE_PROCESSED_STREAM);

            // Enable wireframe mode.
            cameraSim.enableDrawWireframe(Constants.SimulationConstants.USE_WIREFRAME);
        }
    }
    //call periodically
    //does this work?? consult documentation
    fun captureFrame(){
        frame = limelight.allUnreadResults.first()
    }


    // called periodically in a simulation
    fun simPeriodic() {
        // Update with the simulated drivetrain pose. This should be called every loop in simulation.
        visionSim?.update(poseEstimator?.referencePose);
    }


    fun getEstimatedPose(): Optional<EstimatedRobotPose>? =
            poseEstimator?.update(frame)

    //stolen from  photonvision(blatantly)
    fun getEstimationStdDevs(estimatedPose: Pose2d): Matrix<N3, N1> {
        //todo: expiriment with vecbuilder values(somehow)
        var estStdDevs = VecBuilder.fill(.7, .7, .9999999)
        val targets = frame.getTargets()
        var numTags = 0
        var avgDist = 0.0

        for (tgt in targets) {
            val tagPose = poseEstimator?.fieldTags?.getTagPose(tgt.fiducialId) ?: continue
            if (tagPose.isEmpty) continue
            numTags++
            avgDist +=
                    tagPose.get().toPose2d().translation.getDistance(estimatedPose.translation)
        }
        if (numTags == 0) return estStdDevs
        avgDist /= numTags
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = VecBuilder.fill(0.5, 0.5, 2.0)
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE)
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30))

        return estStdDevs
    }


    fun resetPose(pose: Pose2d) {
        poseEstimator ?: return
        poseEstimator?.setReferencePose(pose)
    }
}