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
    public static final CANBus kCANBus = CANConstants.kCANivoreBus;
    
    public static final int kSupplyCurrentLimit = 40;
    public static final int kStatorCurrentLimit = 90;

    public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;

    public static final boolean kUseFOC = true;

    public static final double kGearRatio = 3;

    public static final double kP = 6; //12; // TODO: tune this value
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 4.8;
    public static final double kV = 0.0201; // TODO: find this value
    public static final double kA = 0;

    public static final double kVelocityFilterTimeConstant = 0.01;
}
