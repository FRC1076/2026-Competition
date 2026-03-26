// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.slapdown;

import org.littletonrobotics.junction.AutoLog;

public interface SlapdownIO {
    @AutoLog
    public static class SlapdownIOInputs {
        public double appliedVoltage = 0;
        public double currentAmps = 0;
        public double angleRadians = 0;

        public double velocityRadiansPerSecond = 0;
        public double motorTempDegC = 0;

        public double PIDTargetRadians = 0;
    }

    public abstract void setVoltage(double volts);

    public abstract void setVoltageNoSoftStops(double volts);

    public abstract void setPosition(double radians);

    public default void setPositionStrong(double radians) {
        setPosition(radians);
    }

    public abstract void updateInputs(SlapdownIOInputs inputs);

    public abstract void rezero();

    public default void periodic() {

    }
}