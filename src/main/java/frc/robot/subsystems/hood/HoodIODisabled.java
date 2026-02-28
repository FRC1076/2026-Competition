// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.hood;

public class HoodIODisabled implements HoodIO {
    private double appliedVoltage = 0;
    private double positionTargetRadians = 0;

    @Override
    public void setVoltage(double volts) {
        appliedVoltage = volts;
    }

    @Override
    public void setPosition(double radians) {
        positionTargetRadians = radians;
    }

    @Override
    public void rezero() {
        positionTargetRadians = 0;
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.appliedVolts = appliedVoltage;
        inputs.angleRadians = positionTargetRadians;
    }
}
