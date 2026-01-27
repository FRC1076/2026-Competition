package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
    @AutoLog
    public static class SpindexerIOInputs {
        public double appliedVoltage = 0;
        public double currentAmps = 0;
    }

    public abstract void setVoltage(double volts);

    public abstract void updateInputs(SpindexerIOInputs inputs);

}
