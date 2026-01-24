package frc.robot.subsystems.climber;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CANConstants;

public class ClimberConstants{

    public static final int kCANId = 0;
    public static final CANBus kCANBus = CANConstants.kRioBus;

    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final boolean motorInverted = false;

    public static final double kVoltageCompensation = 12; 
    public static final double kCurrentLimit = 40;

    public static final double kMaxPositionMeters = 1;
    public static final double kMinPositionMeters = 0;
    
    public static final double kGearRatio = 16;
    public static final double kSprocketToothCount = 22;
    public static final double kSprocketPitch = Units.inchesToMeters(1.0/4); // Pitch is the distance between two adjacent teeth

    public static final double kPositionConversionFactor = ((kSprocketToothCount * kSprocketPitch) / kGearRatio);
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;
        


}