// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.roller;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CANConstants;

public class RollerConstants {
    // CAN configs
    public static final int kLeadMotorPort = 41;
    public static final int kFollowMotorPort = 45;
    public static final CANBus kCANBus = CANConstants.kCANivoreBus;

    // Current limits
    public static final int kSupplyCurrentLimit = 40;
    public static final int kStatorCurrentLimit = 60;

    public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;
    public static final MotorAlignmentValue kFollowerAlignment = MotorAlignmentValue.Opposed;
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

    // Gear ratio
    public static final double kGearRatio = 2;

    // Closed-loop
    public static final double kP = 1; // TODO: tune this value
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = (5.0/140.0); // TODO: confirm this value
    public static final double kA = 0;

    // FOC
    public static final boolean kUseFOC = true;
}
