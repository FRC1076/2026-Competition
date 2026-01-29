package frc.robot.subsystems.climber;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import frc.robot.Constants.CANConstants;

public class ClimberConstants{
    public static final int kCANId = 51;
    public static final CANBus kCANBus = CANConstants.kRioBus;

    // PID
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final Constraints kProfileConstraints = new Constraints(4 * Math.PI, 3.5 * Math.PI);

    // Feedforward
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final boolean kMotorInverted = false;

    public static final double kVoltageCompensation = 12; 
    public static final double kCurrentLimit = 40;

    public static final double kMaxPositionMeters = 1;
    public static final double kMinPositionMeters = 0;
    
    public static final double kGearRatio = 16;
    public static final double kDrumRadius = 0.0254;

    public static final double kPositionConversionFactor = (kDrumRadius / kGearRatio);
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;
}