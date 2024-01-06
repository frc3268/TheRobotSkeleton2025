package frc.lib.utils

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import java.io.IOException

class Camera(name:String, path:String){
    val limelight:PhotonCamera = PhotonCamera(name)
    var frame: PhotonPipelineResult = PhotonPipelineResult()
    var poseEstimator: PhotonPoseEstimator? = null

    init {
        try {
            poseEstimator =
                PhotonPoseEstimator(
                    AprilTagFieldLayout(
                        Filesystem.getDeployDirectory().toString() +
                        path
                    ),
                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
                    limelight,
                    Transform3d()
                )
        } catch (e: IOException) {
            DriverStation.reportError("AprilTag: Failed to Load", e.getStackTrace())
        // !add some way to lock down apriltage features after this
        }
    }

    //call periodically
    fun captureFrame() : PhotonPipelineResult{
        frame = limelight.latestResult
        return frame
    }

    fun getArpilTagTargetByID(id: Int):PhotonTrackedTarget? {
        limelight.pipelineIndex = 1
        if(!frame.hasTargets()){
            return null
        }
        for (target:PhotonTrackedTarget in frame.getTargets()){
            if(target.fiducialId == id){
                return target
            }
        }
        return null
    }

    fun getAprilTagTarget():PhotonTrackedTarget? {
        limelight.pipelineIndex = 1
        if(!frame.hasTargets()){
            return null
        }
        return frame.bestTarget
    }

    fun getReflectiveTapeTarget():PhotonTrackedTarget?{
        limelight.pipelineIndex = 0
        if(!frame.hasTargets()){
            return null
        }
        return frame.bestTarget
    }


    fun getEstimatedPose(prevPose: Pose2d): EstimatedRobotPose? {
        poseEstimator ?: return null
        poseEstimator?.setReferencePose(prevPose)
        return poseEstimator?.update()?.orElse(null)
    }

    fun resetPose(pose: Pose2d) {
        poseEstimator ?: return
        poseEstimator?.setReferencePose(pose)
    }
}