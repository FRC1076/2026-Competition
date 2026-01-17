package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public interface HoodIO {
    public static record HoodControlConstants (
        Double kP,
        Double kI,
        Double kD,
        Constraints kProfileConstraints,
        Double kS,
        Double kG,
        Double kV,
        Double kA
    ) {}

    @AutoLog
    public static class HoodIOInputs {
        public double appliedVolts = 0;
        public double leadCurrentAmps = 0;
        public double angleRadians = 0;
        public double velocityRadiansPerSecond = 0;
    }

    public abstract void updateInputs(HoodIOInputs inputs);

    public abstract void setVoltage(double volts);

    public default void simulationPeriodic() {
        
    }
}