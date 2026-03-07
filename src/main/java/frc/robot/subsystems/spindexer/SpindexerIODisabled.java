// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.spindexer;

public class SpindexerIODisabled implements SpindexerIO {
    private double voltageTarget = 0.0;
    private double velocityTarget = 0.0;

    @Override
    public void setVoltage(double volts) {
        voltageTarget = volts;

    }

    @Override
    public void setVelocity(double radPerSec) {
        velocityTarget = radPerSec;
    }

    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        inputs.appliedVoltage = voltageTarget;
        inputs.velocityRadPerSec = velocityTarget;
    }
}
