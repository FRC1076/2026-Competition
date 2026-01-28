package frc.robot.subsystems.slapdown;

public class SlapdownIODisabled implements SlapdownIO {
    private double voltageTarget = 0;

    @Override
    public void setVoltage(double volts) {
        voltageTarget = volts;
    }

    @Override
    public void updateInputs(SlapdownIOInputs inputs) {
        inputs.appliedVoltage = voltageTarget;
    }

    @Override
    public void stop()
    {
        this.setVoltage(0.0);
    }
}