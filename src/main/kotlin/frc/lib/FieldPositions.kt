package frc.lib

import edu.wpi.first.math.geometry.Pose2d

//for clarity, blue is the pose of the field element on the blue side, and red is the same but on the red side
data class FieldLocation(val red: Pose2d, val blue: Pose2d)
object FieldPositions {
    val speakerCenter = FieldLocation(Pose2d(15.256, 5.547868, 0.0.rotation2dFromDeg()),
            Pose2d(1.6096, 5.547868, 180.0.rotation2dFromDeg()))
    val speakerLeft = FieldLocation(Pose2d(15.256, 5.547868, 0.0.rotation2dFromDeg()),
        Pose2d(1.6096, 5.547868, 180.0.rotation2dFromDeg()))
    val speakerRight = FieldLocation(Pose2d(15.256, 5.547868, 0.0.rotation2dFromDeg()),
        Pose2d(1.6096, 5.547868, 180.0.rotation2dFromDeg()))
    fun speakerCloser(startingPose: Pose2d): FieldLocation  =
        //red
       closest(
           startingPose,
           listOf(
               speakerRight,
               speakerCenter,
               speakerLeft
           )
       )

    fun closest(startingPose: Pose2d, locations: List<FieldLocation>): FieldLocation {
        val red = locations.minByOrNull{ startingPose.translation.getDistance(it.red.translation)}!!
        val blue = locations.minByOrNull{ startingPose.translation.getDistance(it.blue.translation)}!!
        return FieldLocation(red.red, blue.blue)

    }

    val amp = FieldLocation(Pose2d(14.929358, 8.2042, 270.0.rotation2dFromDeg()),
            Pose2d(1.84404, 8.2042, 270.0.rotation2dFromDeg()))

    //todo: source, all of the rings
    //val source = FieldLocation
}