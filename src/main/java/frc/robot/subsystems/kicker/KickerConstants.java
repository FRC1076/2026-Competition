// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CANConstants;

public class KickerConstants {
    // CAN Id
    public static final int kLeadMotorCANId = 44;
    public static final int kFollowMotorCANId = 46;
    public static final CANBus kCANBus = CANConstants.kCANivoreBus;

    // PID Constants
    public static final double kP = 3;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 6.0/750.0;
    public static final double kA = 0.0;

    // Voltage compensation and current limit
    public static final double kVoltageCompensation = 12;
    public static final int kSupplyCurrentLimit = 40;
    public static final int kStatorCurrentLimit = 70;

    public static final InvertedValue kPositiveDirection = InvertedValue.Clockwise_Positive;
    public static final MotorAlignmentValue kLeadFollowerAlignment = MotorAlignmentValue.Aligned;
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
    public static final boolean kUseFOC = true;

    // Velocity conversion factor
    public static final double kGearRatio = 5;
    public static final double kVelocityConversionFactor = 2 * Math.PI / kGearRatio / 60.0;
}
