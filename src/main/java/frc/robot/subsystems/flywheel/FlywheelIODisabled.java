package frc.robot.subsystems.flywheel;

public class FlywheelIODisabled implements FlywheelIO {
    private double appliedVoltage;
    private double appliedVelocity;

    public FlywheelIODisabled() {
        // Literally nothing
    }

    @Override
    public void setVoltage(double volts) {
        appliedVoltage = volts;
    }

    @Override
    public void setVelocityRadPerSec(double velocityRadPerSec) {
        appliedVelocity = velocityRadPerSec;
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.appliedLeadVoltage = appliedVoltage;
        inputs.velocityRadiansPerSecond = appliedVelocity;
    }
}
