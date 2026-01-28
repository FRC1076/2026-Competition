package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class SpindexerConstants {
    public static final int kMotorPort = 43;
    
    public static final int kCurrentLimit = 40;

    public static final double kManualShootVolts = 12;
    public static final double kManualReserveVolts = 4;

    public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

    public static class Control {
        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0;
        public static final double kV = 0.025;
        public static final double kA = 0;

        public static final double kMaxAcceleration = 20000;
        public static final double kMaxJerk = 160000;
    }

    public static class ConstrolSim {
        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0;
        public static final double kV = 0.02;
        public static final double kA = 0;
    }
}
