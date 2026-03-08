// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

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
    }

    public abstract void setVoltage(double volts);

    public abstract void setVoltageNoSoftStops(double volts);

    public abstract void setPosition(double positionRadians);

    public abstract void updateInputs(TurretIOInputs inputs);

    public default void setSoftwareStops(boolean enabled) {
        
    }

    public default void resetPosition() {
        resetPositionTo(0);
    }

    public default void resetPositionTo(double position) {

    }

    public default void periodic() {
        
    }
}
