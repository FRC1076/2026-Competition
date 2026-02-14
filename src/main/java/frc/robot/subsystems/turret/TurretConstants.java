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
    public static final double kEncoderOffsetRad = 0;
    public static final double kGearRatio = 10;

    // Closed loop configs
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    // Motion Magic configs
    public static final double kCruiseVelocityRadPerSec = 0;
    public static final double kMaxAccelRadPerSec2 = 0;
    public static final double kMaxJerkRadPerSec3 = 0;

    // FOC
    public static final boolean kEnableFOC = true;

    // Software stops
    public static final double kMaxPositionRad = Math.PI;
    public static final double kMinPositionRad = -Math.PI;

    // SysId
    public static final double kSysIdRampRate = 1;
    public static final double kSysIdStepVoltage = 4;
}
