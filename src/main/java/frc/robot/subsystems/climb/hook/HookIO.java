// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.climb.hook;

import org.littletonrobotics.junction.AutoLog;

public interface HookIO {
    @AutoLog
    public static class HookIOInputs {
        public double appliedVoltage = 0;
        public double currentAmps = 0;
        public double positionRadians = 0;
    }

    public abstract void setVoltage(double volts);

    public abstract void setPosition(double positionMeters);

    public abstract void updateInputs(HookIOInputs inputs);

    public default void periodic() {
        
    }
}