// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.climb.hook;

import com.ctre.phoenix6.CANBus;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.CANConstants;

public class HookConstants {
    public static final int kCANId = 51;
    public static final CANBus kCANBus = CANConstants.kRioBus;

    public static final boolean kMotorInverted = true;

    public static final double kVoltageCompensation = 10; 
    public static final int kCurrentLimit = 20;

    public static final IdleMode kIdleMode = IdleMode.kBrake;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final double kCruiseVelocity = Math.PI;
    public static final double kMaxAccel = Math.PI;

    public static final double kGearRatio = 1;
    public static final double kPositionConversionFactor = (2 * Math.PI) / kGearRatio;
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

    public static final double kHookStowedPosition = 0;
    public static final double kHookOutPosition = Math.PI;

    public static final double kMaxOperatorControlVolts = 2;
}