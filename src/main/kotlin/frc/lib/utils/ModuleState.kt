import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState

class ModuleState {
    /**
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins. Customized from WPILib's version to include placing
     * in appropriate scope for CTRE and REV onboard control as both controllers as of writing don't
     * have support for continuous input.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     */
    fun optimize(desiredState: SwerveModuleState, currentAngle: Rotation2d): SwerveModuleState {
        var targetAngle: Double =
            placeInAppropriate0To360Scope(
                currentAngle.getDegrees(),
                desiredState.angle.getDegrees()
            )
        var targetSpeed: Double = desiredState.speedMetersPerSecond
        val delta: Double = targetAngle - currentAngle.getDegrees()
        if (Math.abs(delta) > 90) {
            targetSpeed *= -1
            targetAngle += if (delta > 90) -180 else +180
        }
        return SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle))
    }

    /**
     * @param scopeReference Current Angle
     * @param newAngle Target Angle
     * @return Closest angle within scope
     */
    fun placeInAppropriate0To360Scope(scopeReference: Double, newAngle: Double): Double {
        val lowerBound: Double
        val upperBound: Double
        var nAngle: Double = newAngle
        val lowerOffset: Double = scopeReference % 360

        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset
            upperBound = scopeReference + (360 - lowerOffset)
        } else {
            upperBound = scopeReference - lowerOffset
            lowerBound = scopeReference - (360 + lowerOffset)
        }

        while (newAngle < lowerBound) {
            nAngle += 360
        }
        while (newAngle > upperBound) {
            nAngle -= 360
        }

        return nAngle + when {
            newAngle - scopeReference > +180 -> -360
            newAngle - scopeReference < -180 -> +360
            else -> 0
        }
    }
}