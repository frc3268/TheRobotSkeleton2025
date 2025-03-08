package frc.lib

import edu.wpi.first.math.geometry.Pose2d

//for clarity, blue is the pose of the field element on the blue side, and red is the same but on the red side
data class FieldLocation(val red: Pose2d, val blue: Pose2d)
data class Obstacle(val location: Pose2d, val radiusMeters: Double)

// TODO: IMPLEMENT! Size is in inches.
// +Z is up into the air from the carpet, +X is horizontal to the right (Based off the image below), +Y runs from the Field Border towards the REEFS.
// For the Z-Rotation (I presume?) 0° faces the red alliance station, 90° faces the non- scoring table side, and 180° faces the blue alliance station.
// For the X-Rotation, 0 is perpendicular to the Z plane, and 90 degrees is facing the carpet. Distances are measured to the center of the tag.
// Distances are measured to the center of the tag.
// AprilTag Coordinates found in page 4 at https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf
// ID X Y Z Z-Rotation X-Rotation Common-Name
// ~~1 657.37 25.80 58.50 126 0 Coral-Station-Bottom-Red~~
// 2 657.37 291.20 58.50 234 0 Coral-Station-Top-Red
// ~~3 455.15 317.15 51.25 270 0 Blue-Processor~~
// 4 365.20 241.64 73.54 0 30 Blue-Barge-Facing-Red
// 5 365.20 75.39 73.54 0 30 Red-Barge-Facing-Red
// 6 530.49 130.17 12.13 300 0 Red-Reef
// 7 546.87 158.50 12.13 0 0 Red-Reef
// 8 530.49 186.83 12.13 60 0 Red-Reef
// 9 497.77 186.83 12.13 120 0 Red-Reef
// 10 481.39 158.50 12.13 180 0 Red-Reef
// 11 497.77 130.17 12.13 240 0 Red-Reef
// ~~12 33.51 25.80 58.50 54 0 Coral-Station-Bottom-Blue~~
// 13 33.51 291.20 58.50 306 0 Coral-Station-Top-Blue
// 14 325.68 241.64 73.54 180 30 Blue-Barge-Facing-Blue
// 15 325.68 75.39 73.54 180 30 Red-Barge-Facing-Blue
// ~~16 235.73 -0.15 51.25 90 0 Red-Processor~~
// 17 160.39 130.17 12.13 240 0 Blue-Reef
// 18 144.00 158.50 12.13 180 0 Blue-Reef
// 19 160.39 186.83 12.13 120 0 Blue-Reef
// 20 193.10 186.83 12.13 60 0 Blue-Reef
// 21 209.49 158.50 12.13 0 0 Blue-Reef
// 22 193.10 130.17 12.13 300 0 Blue-Reef

/** Object used to store position of field elements */
object FieldPositions {

    fun closest(startingPose: Pose2d, locations: List<FieldLocation>): FieldLocation {
        val red = locations.minByOrNull{ startingPose.translation.getDistance(it.red.translation)}!!
        val blue = locations.minByOrNull{ startingPose.translation.getDistance(it.blue.translation)}!!
        return FieldLocation(red.red, blue.blue)
    }

    val coralStationBottom = FieldLocation(
        Pose2d(16.697198, 0.245872, 54.0.rotation2dFromDeg()),
        Pose2d(0.851154, 0.65532, 126.0.rotation2dFromDeg())
    )

    val processor = FieldLocation(
        Pose2d(11.56081, 7.4, 270.0.rotation2dFromDeg()),
        Pose2d(5.987542, 0.762, 90.0.rotation2dFromDeg())
    )
    
    val reef = arrayOf(
        FieldLocation(
           Pose2d(13.474472948945898, 3.306324612649225, 300.0.rotation2dFromDeg()),
           Pose2d(4.073914147828296, 3.306324612649225, 240.0.rotation2dFromDeg())
      ),
        FieldLocation(
           Pose2d(13.890525781051563, 4.025908051816104, 0.0.rotation2dFromDeg()),
           Pose2d(3.6576073152146305, 4.025908051816104, 180.0.rotation2dFromDeg())
      ),
        FieldLocation(
           Pose2d(13.474472948945898, 4.745491490982983, 60.0.rotation2dFromDeg()),
           Pose2d(4.073914147828296, 4.745491490982983, 120.0.rotation2dFromDeg())
      ),
        FieldLocation(
           Pose2d(12.643383286766573, 4.745491490982983, 120.0.rotation2dFromDeg()),
           Pose2d(4.904749809499619, 4.745491490982983, 60.0.rotation2dFromDeg())
      ),
        FieldLocation(
           Pose2d(12.22733045466091, 4.025908051816104, 180.0.rotation2dFromDeg()),
           Pose2d(5.321056642113285, 4.025908051816104, 0.0.rotation2dFromDeg())
      ),
        FieldLocation(
           Pose2d(12.643383286766573, 3.306324612649225, 240.0.rotation2dFromDeg()),
           Pose2d(4.904749809499619, 3.306324612649225, 300.0.rotation2dFromDeg())
      ),
    );

    val reefRightFar = FieldLocation(Pose2d(12.2, 5.5, 300.0.rotation2dFromDeg()), Pose2d(5.4, 2.5, 120.0.rotation2dFromDeg()))
    val reefRightClose = FieldLocation(Pose2d(14.0, 5.5, 240.0.rotation2dFromDeg()), Pose2d(3.6, 2.5, 60.0.rotation2dFromDeg()))
    val reefLeftClose = FieldLocation(Pose2d(14.0, 2.5, 120.0.rotation2dFromDeg()), Pose2d(3.6, 5.5, 300.0.rotation2dFromDeg()))
    val reefLeftFar = FieldLocation(Pose2d(12.2, 2.5, 60.0.rotation2dFromDeg()), Pose2d(5.4, 5.5, 240.0.rotation2dFromDeg()))
    var reefCenterClose = FieldLocation(Pose2d(14.8, 4.0, 180.0.rotation2dFromDeg()), Pose2d(2.75, 4.0, 0.0.rotation2dFromDeg()))

    val sourceLeft = FieldLocation(Pose2d(16.243, 1.0, 306.rotation2dFromDeg()), Pose2d(1.25, 7.0, 126.0.rotation2dFromDeg()))
    val sourceRight = FieldLocation(Pose2d(16.243,6.788, 54.000.rotation2dFromDeg()), Pose2d(1.291, 1.081, 234.0.rotation2dFromDeg()))
}
