package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public double motorAppliedVoltage = 0;
        public double motorCurrentAmps = 0;
        public double motorTempDegC = 0;

        public double motorPositionRad = 0;
        public double motorVelocityRadPerSec = 0;

        public double throughBorePositionRad = 0;
    }

    public abstract void setVoltage(double volts);

    public abstract void setPosition(double positionRadians);

    public abstract void updateInputs(TurretIOInputs inputs);

    public default void setSoftwareStops(boolean enabled) {
        
    }
}
