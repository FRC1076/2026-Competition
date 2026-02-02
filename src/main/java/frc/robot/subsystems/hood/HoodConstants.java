package frc.robot.subsystems.hood;

public class HoodConstants {
    // CAN Id
    public static final int kCANId = 33;

    // Voltage and current limits
    public static final double kSmartCurrentLimit = 10;
    public static final double kMaxOperatorControlVolts = 5;

    // Inverted
    public static final boolean kMotorInverted = false;

    // Software stops and PID tolerance
    public static final double kMaxHoodAngleRadians = 2;
    public static final double kMinHoodAngleRadians = 0;
    public static final double hoodAngleToleranceRadians = 0.1;
    
    // Absolute encoder stuff
    public static final double kEncoderGearRatio = 125;
    public static final double kPositionConversionFactor = 2 * Math.PI * kEncoderGearRatio;
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60;
    public static final double kZeroOffsetRadians = 0;

    // PID
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kMaxVelocityRadPerSec = Math.PI;
    public static final double kMaxAccelerationRadPerSec2 = 2*Math.PI;

    // Feedforward
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kA = 0;
}