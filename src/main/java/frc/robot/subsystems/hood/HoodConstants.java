// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.hood;

public class HoodConstants {
    // CAN Id
    public static final int kCANId = 33;

    // Voltage and current limits
    public static final double kSmartCurrentLimit = 10;
    public static final double kMaxOperatorControlVolts = 3;

    // Inverted
    public static final boolean kMotorInverted = true;

    // Software stops and PID tolerance
    public static final double kMaxHoodAngleRadians = 0.25;
    public static final double kMinHoodAngleRadians = 0;
    public static final double hoodAngleToleranceRadians = 0.1;
    
    // Absolute encoder stuff
    public static final double kEncoderGearRatio = 300.0/16.0;
    public static final double kPositionConversionFactor = (2 * Math.PI) / kEncoderGearRatio;
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60;
    public static final double kZeroOffsetRadians = 0;

    // Relative encoder stuff
    public static final double kRelativeEncoderAdditionalGearRatio = 25;
    public static final double kPositionRelEncoderConversionFactor = (2* Math.PI) / (kRelativeEncoderAdditionalGearRatio * kEncoderGearRatio);
    public static final double kVelocityRelEncoderConversionFactor = kPositionRelEncoderConversionFactor / 60;

    // PID
    public static final double kP = 50;
    public static final double kI = 0;
    public static final double kD = 2;

    // Feedforward
    public static final double kS = 0;
    public static final double kCos = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final double kCosRatio = 1.0/(2.0*Math.PI);

    public static final double kCruiseVelocity = 100*Math.PI;
    public static final double kMaxAccel = 100*Math.PI;
}