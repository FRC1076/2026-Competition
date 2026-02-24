// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.climb.hook;

public class HookIODisabled implements HookIO {
    private double appliedVoltage = 0;
    private double targetPosition = 0;

    public HookIODisabled() {
        // Literally nothing
    }

    @Override
    public void setVoltage(double volts) {
        appliedVoltage = volts;
    }

    @Override
    public void setPosition(double positionRadians) {
        targetPosition = positionRadians;
    }

    @Override
    public void updateInputs(HookIOInputs inputs) {
        inputs.appliedVoltage = appliedVoltage;
        inputs.positionRadians = targetPosition;
    }
}