package frc.robot.subsystems.roller;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CANConstants;

public class RollerConstants {
    public static final int kMotorPort = 41;
    public static final CANBus kCANBus = CANConstants.kCANivoreBus;

    public static final int kSupplyCurrentLimit = 30;
    public static final int kStatorCurrentLimit = 40;

    public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
}
