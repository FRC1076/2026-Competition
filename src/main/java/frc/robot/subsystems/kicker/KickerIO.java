package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
    @AutoLog
    public static class KickerIOInputs {
        public double appliedVoltage = 0;
        public double currentAmps = 0;
        
        public double velocityRadPerSec = 0;
    }

    public abstract void setVoltage(double volts);

    public abstract void updateInputs(KickerIOInputs inputs);
}
