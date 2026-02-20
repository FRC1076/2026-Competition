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

    public static final double kGearRatio = 25;
    public static final double kRotorOffsetRot = -0.241699 + 0.014648 + 0.013916;

    public static final double kP = 3;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kV = 0;
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kA = 0;

    public static final double kCruiseVelocityRadPerSec = 4 * Math.PI;
    public static final double kMaxAccelRadPerSec2 = 3.5 * Math.PI;
    public static final double kMaxJerkRadPerSec3 = 100 * Math.PI;

    public static final double kAngleToleranceRadians = 0.1;
    public static final double kMinAngleRadians = 0;
    public static final double kMaxAngleRadians = 5.5;

    public static final double kMaxOperatorControlVolts = 1;

}
