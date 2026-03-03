package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.led.LEDConstants.LEDOnRIOConstants;
import frc.robot.subsystems.led.LEDConstants.LEDStates;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Percent;

public class LEDIORio implements LEDBase {
    private final AddressableLED m_leds;
    private final AddressableLEDBuffer m_buffer;

    private LEDStates currentState = LEDStates.OFF;

    private final LEDPattern m_off = LEDPattern.kOff;

    private final LEDPattern m_purple = LEDPattern.solid(Color.kPurple)
        .atBrightness(Percent.of(LEDOnRIOConstants.kBrightnessPercentage));

    private final LEDPattern m_purpleGradient = LEDPattern.gradient(
        LEDPattern.GradientType.kContinuous,
        Color.kPurple,
        Color.kWhite
    ).scrollAtRelativeSpeed(Hertz.of(0.5))
        .atBrightness(Percent.of(LEDOnRIOConstants.kBrightnessPercentage));

    private final LEDPattern m_rainbow = LEDPattern.rainbow(
        255, 255
    ).scrollAtRelativeSpeed(Hertz.of(0.5))
        .atBrightness(Percent.of(LEDOnRIOConstants.kBrightnessPercentage));


    public LEDIORio() {
        m_leds = new AddressableLED(LEDOnRIOConstants.kPWMPort);
        m_buffer = new AddressableLEDBuffer(LEDOnRIOConstants.kLength);

        // Setting the length is intensive, so ONLY update data after this
        m_leds.setLength(m_buffer.getLength());

        m_off.applyTo(m_buffer); // Start at off
        m_leds.setData(m_buffer);
        m_leds.start();
    }

    @Override
    public void setState(LEDStates state) {
        currentState = state;

        if (state == LEDStates.OFF) {
            m_off.applyTo(m_buffer);
            m_leds.setData(m_buffer);
        } else if (state == LEDStates.PURPLE_WHITE_GRADIENT) {
            m_purpleGradient.applyTo(m_buffer);
            m_leds.setData(m_buffer);
        } else if (state == LEDStates.RAINBOW) {
            m_rainbow.applyTo(m_buffer);
            m_leds.setData(m_buffer);
        } else {
            m_purple.applyTo(m_buffer);
            m_leds.setData(m_buffer);
        }
    }

    @Override
    public LEDStates getState() {
        return currentState;
    }
}