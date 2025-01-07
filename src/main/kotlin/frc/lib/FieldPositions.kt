package frc.lib

import edu.wpi.first.math.geometry.Pose2d
import java.util.function.Supplier

//for clarity, blue is the pose of the field element on the blue side, and red is the same but on the red side
data class FieldLocation(val red: Pose2d, val blue: Pose2d)
data class Obstacle(val location: Pose2d, val radiusMeters: Double)

// TODO: IMPLEMENT! Size is in inches.
// +Z is up into the air from the carpet, +X is horizontal to the right (Based off the image below), +Y runs from the Field Border towards the REEFS.
//For the Z-Rotation (I pressume?) 0° faces the red alliance station, 90° faces the non- scoring table side, and 180° faces the blue alliance station. 
// For the X-Rotation, 0 is perpendicular to the Z plane, and 90 degrees is facing the carpet. Distances are measured to the center of the tag.
// Distances are measured to the center of the tag.
// AprilTag Cordinates found in page 4 at https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf
// ID X Y Z Z-Rotation X-Rotation
// 1 657.37 25.80 58.50 126 0
// 2 657.37 291.20 58.50 234 0
// 3 455.15 317.15 51.25 270 0
// 4 365.20 241.64 73.54 0 30
// 5 365.20 75.39 73.54 0 30
// 6 530.49 130.17 12.13 300 0
// 7 546.87 158.50 12.13 0 0
// 8 530.49 186.83 12.13 60 0
// 9 497.77 186.83 12.13 120 0
// 10 481.39 158.50 12.13 180 0
// 11 497.77 130.17 12.13 240 0
// 12 33.51 25.80 58.50 54 0
// 13 33.51 291.20 58.50 306 0
// 14 325.68 241.64 73.54 180 30
// 15 325.68 75.39 73.54 180 30
// 16 235.73 -0.15 51.25 90 0
// 17 160.39 130.17 12.13 240 0
// 18 144.00 158.50 12.13 180 0
// 19 160.39 186.83 12.13 120 0
// 20 193.10 186.83 12.13 60 0
// 21 209.49 158.50 12.13 0 0
// 22 193.10 130.17 12.13 300 0

object FieldPositions {
    val speakerCenter = FieldLocation(Pose2d(15.256, 5.547868, 0.0.rotation2dFromDeg()),
            Pose2d(1.6096, 5.547868, 180.0.rotation2dFromDeg()))
    val speakerLeft = FieldLocation(Pose2d(15.256, 5.547868, 0.0.rotation2dFromDeg()),
        Pose2d(1.6096, 5.547868, 180.0.rotation2dFromDeg()))
    val speakerRight = FieldLocation(Pose2d(15.256, 5.547868, 0.0.rotation2dFromDeg()),
        Pose2d(1.6096, 5.547868, 180.0.rotation2dFromDeg()))

    fun closest(startingPose: Pose2d, locations: List<FieldLocation>): FieldLocation {
        val red = locations.minByOrNull{ startingPose.translation.getDistance(it.red.translation)}!!
        val blue = locations.minByOrNull{ startingPose.translation.getDistance(it.blue.translation)}!!
        return FieldLocation(red.red, blue.blue)

    }

    val amp = FieldLocation(
            Pose2d(14.929358, 8.2042, 270.0.rotation2dFromDeg()),
            Pose2d(1.84404, 8.2042, 270.0.rotation2dFromDeg()))

    val sourceBaseline = FieldLocation(
        Pose2d(0.356108, 0.883666, 60.0.rotation2dFromDeg()),
        Pose2d(15.079472, 0.245872, 120.0.rotation2dFromDeg())

    )

    val sourceNotBaseline = FieldLocation(
        Pose2d(1.461516, 0.245872, 60.0.rotation2dFromDeg()),
        Pose2d(16.185134, 0.883666, 120.0.rotation2dFromDeg())
    )

    //obstacles do need to be iterated through, but they do not need to have names
    val obstacles:Array<Obstacle> = arrayOf(
        Obstacle(Pose2d(4.6, 4.2, 0.0.rotation2dFromDeg()), 1.0)
        //list obstacles here...
    )

}
