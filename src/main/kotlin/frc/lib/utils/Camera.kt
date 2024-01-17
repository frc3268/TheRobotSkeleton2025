package frc.lib.utils

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.lib.basics.SwerveDriveBase
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import java.io.IOException

class Camera(name:String, path:String, val drive:SwerveDriveBase): SubsystemBase(){
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
                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    limelight,
                    Transform3d()
                )
        } catch (e: IOException) {
            DriverStation.reportError("AprilTag: Failed to Load", e.getStackTrace())
        // !add some way to lock down apriltage features after this
        }
    }

    override fun periodic() {
        captureFrame()
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

    // Old code in case it is needed
    //fun getEstimatedPose(prevPose: Pose2d): EstimatedRobotPose? {
    //    poseEstimator ?: return null
    //    poseEstimator?.setReferencePose(prevPose)
    //    return poseEstimator?.update()?.orElse(null)
    //}
    // New code (hope this works and is what you meant
    fun getEstimatedPose(leftDist:Double, rightDist:Double) {
        //Gives me an error every time I attempt to use it
        //poseEstimator.update(drive.getYaw(), leftDist, rightDist);
        var res = limelight.latestResult;
        if(res.hasTargets()) {
            var imageCaptureTime = res.timestampSeconds;
            var camToTargetTrans = res.bestTarget.bestCameraToTarget;
            // need an equivilant to kfartargetpose here
            var camPose = Constants.kFarTargetPose.transformBy(camToTargetTrans.inverse());
            // need an equal to add vision measurement for poseEstimator
            poseEstimator?.update(res)
        }
    }

    fun resetPose(pose: Pose2d) {
        poseEstimator ?: return
        poseEstimator?.setReferencePose(pose)
    }
}