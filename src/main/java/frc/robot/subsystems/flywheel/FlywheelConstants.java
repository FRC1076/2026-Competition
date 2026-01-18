package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CANConstants;

public class FlywheelConstants {
    // General constants here
    public static final int kCANId = 0;
    public static final CANBus kCANBus = CANConstants.kCANivoreBus;

    // Voltage and current limits
    public static final double kMaxVoltage = 12;
    public static final double kStatorCurrentLimit = 60;
    public static final double kSupplyCurrentLimit = 40;

    public static final double kManualFlywheelVolts = 12;
    public static final double kManualReverseVolts = 4;

    public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;


    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kS = 0;
    public static final double kV = 0.025;
    public static final double kA = 0;

    public static final double kMaxAcceleration = 2000;
    public static final double kMaxJerk = 160000;

    public static final boolean kEnableFoc = true;
}

