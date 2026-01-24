package frc.robot.subsystems.climber;

public class ClimberIODisabled implements ClimberIO {
    private double appliedVoltage = 0;
    private double targetPosition = 0;

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
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.appliedVoltage = appliedVoltage;
        inputs.climberPosition = targetPosition;
    }

    @Override
    public void stop() {

    }
}