// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.climb.climber;

import com.ctre.phoenix6.CANBus;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import frc.robot.Constants.CANConstants;

public class ClimberConstants {
    public static final int kCANId = 52;
    public static final CANBus kCANBus = CANConstants.kRioBus;

    public static final boolean kMotorInverted = false;

    public static final IdleMode kIdleMode = IdleMode.kBrake;

    public static final double kVoltageCompensation = 12; 
    public static final int kCurrentLimit = 40;

    // PID
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final Constraints kProfileConstraints = new Constraints(1, 2);

    // Feedforward
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final double kMaxPositionMeters = 1;
    public static final double kMinPositionMeters = 0;
    
    public static final double kGearRatio = 16;
    public static final double kDrumRadius = 0.0254;

    public static final double kPositionConversionFactor = ((2 * Math.PI * kDrumRadius) / kGearRatio);
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

    public static final double kClimberUpPosition = 0.5;
    public static final double kClimberDownPosition = 0;

    public static final double kMaxOperatorControlVolts = 6;
}