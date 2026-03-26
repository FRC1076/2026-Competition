// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.slapdown;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CANConstants;

public class SlapdownConstants {
    public static final int kCANId = 42;
    public static final CANBus kCANBus = CANConstants.kCANivoreBus;

    public static final double kSupplyCurrentLimit = 40;
    public static final double kStatorCurrentLimit = 40;

    public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

    public static final boolean kUseFOC = true;

    public static final double kGearRatio = 25 * (48.0/32.0);
    public static final double kRotorOffsetRot = -0.307617;

    public static final double kP = 12;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kV = 0;
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kA = 0;

    public static final double kCruiseVelocityRadPerSec = 40 * Math.PI;
    public static final double kMaxAccelRadPerSec2 = 60 * Math.PI;
    public static final double kMaxJerkRadPerSec3 = 1000 * Math.PI;

    public static final double kAngleToleranceRadians = 0.1;
    public static final double kMinAngleRadians = 0;
    public static final double kMaxAngleRadians = 5.5;

    public static final double kPStrong = 20;

    public static final double kMaxOperatorControlVolts = 1;

}
