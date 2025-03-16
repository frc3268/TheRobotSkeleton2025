package frc.lib.led

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

/** Create an LedStrip controller
 *
 * @param[ledPort] The output port to use. Must be a PWM header, not MXP or DIO
 * @param[ledLength] The length of the LED buffer in pixels.
 */
class LedStrip(
    val ledPort: Int,
    val ledLength: Int
) : SubsystemBase() {

    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    val ledStrip = AddressableLED(ledPort)
    val ledBuffer = AddressableLEDBuffer(ledLength)

    init {
        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        ledStrip.setLength(ledBuffer.length)

        // Set the data
        ledStrip.setData(ledBuffer)
        ledStrip.start()

        setDefaultCommand(applyPattern(LEDPattern.solid(Color.kBlack)).withName("Off"))
    }

    /** Apply a [pattern] to the [LedStrip].
     * @param[pattern] the pattern to apply to the buffer
     * @sample applyGradientPattern
     */
    fun applyPattern(pattern: LEDPattern): Command = run {
        pattern.applyTo(ledBuffer)
    }

    /** Apply a gradient pattern to the [LedStrip]. */
    fun applyGradientPattern(color1: Color, color2: Color): Command = runOnce {
        val gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, color1, color2)
        applyPattern(gradient)
    }

    override fun periodic() {
        // Periodically send the latest LED color data to the LED strip for it to display
        ledStrip.setData(ledBuffer)
    }
}