package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double appliedVoltage = 0;
        public double currentAmps = 0;

        public double positionMeters = 0;
        public double velocityMPS = 0;
    }

    public abstract void setVoltage(double volts);

    public abstract void setPosition(double positionMeters);

    public abstract void stop();

    public abstract void updateInputs(ClimberIOInputs inputs);

    public default void periodic() {
        
    }
}