package frc.robot.subsystems.turret;

public class TurretIODisabled implements TurretIO {
    private double appliedVoltage = 0;
    private double targetPosition = 0;

    public TurretIODisabled() {
        // does nothing
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
    public void updateInputs(TurretIOInputs inputs) {
        inputs.motorAppliedVoltage = appliedVoltage;
        inputs.motorPositionRad = targetPosition;
    }
}
