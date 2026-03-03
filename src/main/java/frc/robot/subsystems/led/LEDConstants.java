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
        public static final int kLength = 72;

        public static final double kFlashSeconds = 0.1;
        public static final int kEmptyStateBrightness = 100;
        public static final int kFlashingStateBrightness = 100;
    }

    public static enum LEDStates {
        IDLE,
        CORAL_INDEXED,
        HUMAN_PLAYER_SIGNAL,
        ALGAE,
        AUTO_ALIGNED,
        AUTO_ALIGNING,
        OFF,
        ELEVATOR_ZEROED,
        RED_HP_SIGNAL,
        ORANGE_HP_SIGNAL,
        YELLOW_HP_SIGNAL,
        GREEN_HP_SIGNAL,
        BLUE_HP_SIGNAL,
        PURPLE_HP_SIGNAL,
        WHITE_HP_SIGNAL;
    }
}
