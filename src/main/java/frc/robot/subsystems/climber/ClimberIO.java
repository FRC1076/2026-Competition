package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double appliedVoltage = 0;
        public double currentAmps = 0;
        public double appliedOutput = 0;

        public double positionMeters = 0;
        public double velocityMPS = 0;
    }

    public abstract void setVoltage(double volts);

    public abstract void setPosition(double positionMeters);

    public abstract void updateInputs(ClimberIOInputs inputs);

    public abstract void stop();

}