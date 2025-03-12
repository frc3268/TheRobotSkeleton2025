package frc.lib.led

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.LEDPattern

/** Create an LedStrip controller
 *
 * @param[ledPort] The output port to use. Must be a PWM header, not MXP or DIO
 * @param[ledLength] The length of the LED buffer in pixels.
 */
class LedStrip(
    val ledPort: Int,
    val ledLength: Int
) {

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
    }

    fun applyPattern(pattern: LEDPattern) {
        pattern.applyTo(ledBuffer)
        ledStrip.setData(ledBuffer);
    }
}