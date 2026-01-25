package frc.robot.subsystems.climber;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import frc.robot.Constants.CANConstants;

public class ClimberConstants{

    public static final int kCANId = 0;
    public static final CANBus kCANBus = CANConstants.kRioBus;

    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final Constraints kProfileConstraints = new Constraints(4 * Math.PI, 3.5 * Math.PI);

    public static final boolean motorInverted = false;

    public static final double kVoltageCompensation = 12; 
    public static final double kCurrentLimit = 40;

    public static final double kMaxPositionMeters = 1;
    public static final double kMinPositionMeters = 0;
    
    public static final double kGearRatio = 16;
    public static final double kDrumRadius = 0.0254;

    public static final double kPositionConversionFactor = (kDrumRadius / kGearRatio);
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;
        


}