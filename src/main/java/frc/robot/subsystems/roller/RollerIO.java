// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

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

    public abstract void setVelocity(double radPerSec);
    
    public abstract void updateInputs(RollerIOInputs inputs);
}
