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
    public void updateInputs(HoodIOInputs inputs) {
        inputs.appliedVolts = appliedVoltage;
        inputs.angleRadians = positionTargetRadians;
    }
}
