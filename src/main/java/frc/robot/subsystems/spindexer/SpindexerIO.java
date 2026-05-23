// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DriverStation;

public interface SpindexerIO {
    @AutoLog
    public static class SpindexerIOInputs {
        public double appliedVoltage = 0;
        public double currentAmps = 0;
        public double velocityRadPerSec = 0;
        public double motorTempDegC = 0;
    }

    public abstract void setVoltage(double volts);

    public abstract void setVelocity(double radPerSec);

    public default void setTorque(double amps) {
        DriverStation.reportError("Torque Control Not Supported On This Spindexer IO Implementation.", true);
    }

    public abstract void updateInputs(SpindexerIOInputs inputs);

}
