package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CANConstants;

public class FlywheelConstants {
    // General constants here
    public static final int kLeadMotorCANId = 31;
    public static final int kFollowMotorCANId = 32;
    public static final CANBus kCANBus = CANConstants.kCANivoreBus;

    // Voltage and current limits
    public static final double kMaxVoltage = 12;
    public static final double kStatorCurrentLimit = 60;
    public static final double kSupplyCurrentLimit = 40;

    public static final double kManualFlywheelVolts = 12;
    public static final double kManualReverseVolts = 4;

    public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;


    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final double kMaxAcceleration = 20000;
    public static final double kMaxJerk = 1600000;

    public static final double kSetpointToleranceRadPerSec = 25;

    public static final boolean kEnableFOC = true;
}

