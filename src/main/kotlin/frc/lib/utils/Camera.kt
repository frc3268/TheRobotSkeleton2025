package frc.lib.utils

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
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
import java.util.*

class Camera(name:String, path:String): SubsystemBase(){
    val limelight:PhotonCamera = PhotonCamera(name)
    var frame: PhotonPipelineResult = PhotonPipelineResult()
    var poseEstimator: PhotonPoseEstimator? = null


    init {
        try {
            poseEstimator =
                PhotonPoseEstimator(
                    AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    limelight,
                    //todo: this!
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
    
    // New code (hope this works and is what you meant
    fun getEstimatedPose(): Optional<EstimatedRobotPose>?  {
        return poseEstimator?.update()
    }

    //stolen from  photonvision(blatantly)
    fun getEstimationStdDevs(estimatedPose: Pose2d): Matrix<N3, N1> {
        //todo: expiriment with vecbuilder values(somehow)
        var estStdDevs =  VecBuilder.fill(4.0, 4.0, 8.0)
        val targets = captureFrame().getTargets();
        var numTags = 0;
        var avgDist:Double = 0.0;
        for (tgt in targets) {
            val tagPose = poseEstimator?.getFieldTags()?.getTagPose(tgt.getFiducialId()) ?: continue;
            if (tagPose.isEmpty) continue
            numTags++;
            avgDist +=
                tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = VecBuilder.fill(0.5, 0.5, 1.0);
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }


    fun resetPose(pose: Pose2d) {
        poseEstimator ?: return
        poseEstimator?.setReferencePose(pose)
    }
}