// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs {
        public double appliedVolts = 0;
        public double currentAmps = 0;
        public double angleRadians = 0;
        public double velocityRadiansPerSecond = 0;
    }

    public abstract void updateInputs(HoodIOInputs inputs);

    public abstract void setVoltage(double volts);

    public abstract void setPosition(double radians);

    public default void periodic() {
        
    }
}