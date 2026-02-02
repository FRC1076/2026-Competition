package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public double appliedLeadVoltage = 0;
        public double leadCurrentAmps = 0;
        public double leadTemperatureDegC = 0;

        public double followAppliedVoltage = 0;
        public double followCurrentAmps = 0;
        public double followTemperatureDegC = 0;

        public double velocityRadiansPerSecond = 0;

        
    }

    public abstract void updateInputs(FlywheelIOInputs inputs);

    public abstract void setVoltage(double volts);

    public abstract void setVelocityRadPerSec(double velocityRadPerSec);
}