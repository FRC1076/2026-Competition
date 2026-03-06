package frc.robot.subsystems.led;

public class LEDConstants {
    public static final double kHPSignalTime = 3.0;

    public static class LEDDIOConstants {
        public static final int kDIOPort1 = 7;
        public static final int kDIOPort2 = 8;
        public static final int kDIOPort3 = 9; 
    }

    public static class LEDOnRIOConstants {
        public static final int kPWMPort = 0;
        public static final int kLength = 87;

        public static final double kFlashSeconds = 0.1;
        public static final int kBrightnessPercentage = 40;
    }

    public static enum LEDStates {
        OFF,
        PURPLE_WHITE_GRADIENT,
        RAINBOW;
    }
}
