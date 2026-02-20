// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.slapdown;

public class SlapdownIODisabled implements SlapdownIO {
    private double voltageTarget = 0;
    private double positionTarget = 0;

    @Override
    public void setVoltage(double volts) {
        voltageTarget = volts;
    }

    @Override
    public void setVoltageNoSoftStops(double volts) {
        setVoltage(volts);
    }

    @Override
    public void setPosition(double radians) {
        positionTarget = radians;
    }

    @Override
    public void updateInputs(SlapdownIOInputs inputs) {
        inputs.appliedVoltage = voltageTarget;
        inputs.angleRadians = positionTarget;
    }
}