// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CANConstants;

public class SpindexerConstants {
    public static final int kCANId = 43;
    public static final CANBus kCANBus = CANConstants.kRioBus;
    
    public static final int kSupplyCurrentLimit = 40;
    public static final int kStatorCurrentLimit = 60;

    public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

    public static final boolean kUseFOC = true;

    public static final double kGearRatio = 9;

    public static final double kP = 6; // TODO: tune this value
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 5.0/68.0; // TODO: find this value
    public static final double kA = 0;
}
