package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog

    public static class ClimberIOInputs {

        public double appliedVoltage = 0;
        public double currentAmps = 0;

        public double velocityRadiansPerSecond = 0;

        public double temperatureCelsius = 0;
        public double motorCurrent = 0;

        public double motorPositionMeters = 0;
        public double motorVelocityRadPerSec = 0;
    }

    public abstract void setVoltage(double volts);

    public abstract void setPosition(double positionMeters);

    public abstract void updateInputs(ClimberIOInputs inputs);
}