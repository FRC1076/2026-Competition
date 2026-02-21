// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CANConstants;

public class TurretConstants {
    // CAN
    public static final int kCANId = 34;
    public static final CANBus kCANBus = CANConstants.kCANivoreBus;

    // Voltage and current limits
    public static final double kMaxVoltage = 12;
    public static final double kStatorCurrentLimitAmps = 40;
    public static final double kSupplyCurrentLimitAmps = 30;

    // Manual control
    public static final double kMaxManualControlVolts = 1.5;

    // If inverted and brake mode
    public static final InvertedValue kInvertedValue = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

    // Internal absolute encoder stuff
    public static final double kEncoderOffsetRot = -0.157227;
    public static final double kGearRatio = 10;

    // Closed loop configs
    public static final double kP = 10;
    public static final double kI = 0;
    public static final double kD = 0.5;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    // Motion Magic configs
    public static final double kCruiseVelocityRadPerSec = 4*Math.PI;
    public static final double kMaxAccelRadPerSec2 = 100*Math.PI;
    public static final double kMaxJerkRadPerSec3 = 100000*Math.PI;

    // FOC
    public static final boolean kEnableFOC = true;

    // Software stops
    public static final double kMaxPositionRad = 4.7 - ((4.7+1.7) - 2*Math.PI); // Cutting off a little bit of extra range of motion so wrapping works
    public static final double kMinPositionRad = -1.7;
    public static final double kAngleRange = kMaxPositionRad - kMinPositionRad;

    // SysId
    public static final double kSysIdRampRate = 1;
    public static final double kSysIdStepVoltage = 4;
}
