// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.kicker;

public class KickerIODisabled implements KickerIO {
    private double voltageTarget = 0.0; 
    
    @Override
    public void setVoltage(double volts) {
        voltageTarget = volts;
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        inputs.leadMotorAppliedVoltage = voltageTarget;
    }
    
    @Override
    public void setVelocityRadPerSec(double velocity) {
        // Do nothing, we are disabled
    }
}
