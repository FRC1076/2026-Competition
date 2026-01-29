package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public double appliedVoltage = 0;
        public double currentAmps = 0;

        public double velocityRadiansPerSecond = 0;

        public double temperatureDegC = 0;
    }

    public abstract void updateInputs(FlywheelIOInputs inputs);

    public abstract void setVoltage(double volts);

    public abstract void setVelocityRadPerSec(double velocityRadPerSec);
}