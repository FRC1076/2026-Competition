// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
    @AutoLog
    public static class KickerIOInputs {
        public double leadMotorAppliedVoltage = 0;
        public double followMotorAppliedVoltage = 0;

        public double leadMotorCurrentAmps = 0;
        public double followMotorCurrentAmps = 0;
        
        public double leadMotorVelocityRadPerSec = 0;
        public double followMotorVelocityRadPerSec = 0;
    }

    public abstract void setVoltage(double volts);

    public abstract void setVelocityRadPerSec(double velocity);

    public abstract void updateInputs(KickerIOInputs inputs);
}
