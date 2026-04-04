// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
    @AutoLog
    public static class RollerIOInputs {
        public double leadMotorAppliedVoltage = 0;
        public double followMotorAppliedVoltage = 0;

        public double leadMotorCurrentAmps = 0;
        public double followMotorCurrentAmps = 0;

        public double leadMotorVelocityRadPerSec = 0;
        public double followMotorVelocityRadPerSec = 0;

        public double leadMotorTempDegC = 0;
        public double followMotorTempDegC = 0;
    }

    public abstract void setVoltage(double volts);

    public abstract void setVelocity(double radPerSec);
    
    public abstract void updateInputs(RollerIOInputs inputs);
}
