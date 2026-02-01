package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
    @AutoLog
    public static class RollerIOInputs {
        public double appliedVoltage = 0;
        public double currentAmps = 0;
        public double velocityRadPerSec;
        public double motorTempDegC = 0;
    }

    public abstract void setVoltage(double volts);
    
    public abstract void updateInputs(RollerIOInputs inputs);

}
