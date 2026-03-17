// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.CANConstants;

public class KickerConstants {
    // CAN Id
    public static final int kCanId = 44;
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
    public static final int kStatorCurrentLimit = 50;

    // NEO
    public static final boolean kInverted = true;
    public static final IdleMode kIdleMode = IdleMode.kBrake;

    // KRAKEN
    public static final InvertedValue kPositiveDirection = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

    // Velocity conversion factor
    public static final double kGearRatio = 5;
    public static final double kVelocityConversionFactor = 2 * Math.PI / kGearRatio / 60.0;
}
