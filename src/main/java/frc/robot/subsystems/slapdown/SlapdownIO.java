package frc.robot.subsystems.slapdown;

import org.littletonrobotics.junction.AutoLog;

public interface SlapdownIO {
    @AutoLog
    public static class SlapdownIOInputs {

        public double appliedVoltage = 0;
        public double currentAmps = 0;
        public double appliedOutput = 0;

        public double motorCurrent = 0;

        public double angleRadians = 0;
        public double velocityRadiansPerSecond = 0;
    }

    public abstract void setVoltage(double volts);

    public abstract void setPosition(double radians);

    public abstract void updateInputs(SlapdownIOInputs inputs);

    public abstract void stop();

}