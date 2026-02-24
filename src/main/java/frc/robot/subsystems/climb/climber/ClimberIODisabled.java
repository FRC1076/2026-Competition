// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.climb.climber;

public class ClimberIODisabled implements ClimberIO {
    private double appliedVoltage = 0;
    private double targetPosition = 0;

    private double hookAppliedVoltage = 0;
    private double hookTargetPosition = 0;

    public ClimberIODisabled() {
        // Literally nothing
    }

    @Override
    public void setVoltage(double volts) {
        appliedVoltage = volts;
    }

    @Override
    public void setPosition(double positionMeters) {
        targetPosition = positionMeters;
    }

    @Override
    public void setHookVoltage(double volts) {
        hookAppliedVoltage = volts;
    }

    @Override
    public void setHookPosition(double positionRadians) {
        hookTargetPosition = positionRadians;
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.appliedVoltage = appliedVoltage;
        inputs.positionMeters = targetPosition;

        inputs.hookAppliedVoltage = hookAppliedVoltage;
        inputs.hookPositionRadians = hookTargetPosition;
    }
}