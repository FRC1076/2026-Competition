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
    public static final int kStatorCurrentLimit = 50;

    public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

    public static final double kGearRatio = 1;
}
