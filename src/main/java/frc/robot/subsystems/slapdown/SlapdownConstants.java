package frc.robot.subsystems.slapdown;

import com.ctre.phoenix6.CANBus;
import frc.robot.Constants.CANConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class SlapdownConstants {
    public static final int kCANId = 42;
    public static final CANBus kCANBus = CANConstants.kRioBus;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kV = 0;
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kA = 0;

    public static final Constraints kProfileConstraints = new Constraints(4 * Math.PI, 3.5 * Math.PI);

    public static final double kAngleToleranceRadians = 0.1;
    public static final double kMinAngleRadians = -Math.PI / 4;
    public static final double kMaxAngleRadians = Math.PI / 2;

    public static final double maxOperatorControlVolts = 1;
    public static final double kSmartCurrentLimit = 40.0;

    public static final boolean kMotorInverted = false;

    public static final double kGearRatio = 25;
    public static final double kPositionConversionFactor = 2 * Math.PI / kGearRatio;
    public static final double kVelocityConversionFactor = 2 * Math.PI / 60;
    public static final double kInitialPosition = Math.PI / 2;
}
