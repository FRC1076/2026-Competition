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
    public static final InvertedValue kInvertedValue = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

    // Internal absolute encoder stuff
    public static final double kEncoderOffsetRot = -0.218262;
    public static final double kGearRatio = 10;

    // Starting turret position
    public static final double kStartingPositionOffsetRad = -Math.PI/4.0;

    // Closed loop configs
    public static final double kP = 80;
    public static final double kI = 0;
    public static final double kD = 2;
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
    public static final double kMaxPositionRad = 2*Math.PI;
    public static final double kMinPositionRad = -2*Math.PI;

    // SysId
    public static final double kSysIdRampRate = 1;
    public static final double kSysIdStepVoltage = 4;
}
