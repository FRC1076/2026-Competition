package frc.robot.subsystems.turret;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TurretConstants {
    // Device ID
    public static final int kCanId = 0;

    // Voltage and current limits
    public static final double kMaxVoltage = 12;
    public static final double kCurrentLimitAmps = 40;

    // If inverted and brake mode
    public static final InvertedValue kInvertedValue = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

    // Closed loop configs
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    // Motion Magic configs
    public static final double kCruiseVelocityRadPerSec = 0;
    public static final double kMaxAccelRadPerSec2 = 0;
    public static final double kMaxJerkRadPerSec3 = 0;

    // FOC
    public static final boolean kEnableFOC = false;
}
