package frc.robot.subsystems.slapdown;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class SlapdownIODisabled implements SlapdownIO {
    private final SlapdownControlConstants disabledConstants = new SlapdownControlConstants(
        0.0, 0.0, 0.0, new Constraints(0, 0), 
        0.0, 0.0, 0.0, 0.0);
    private double voltageTarget = 0;

    @Override
    public void setVoltage(double volts) {
        voltageTarget = volts;
    }

    @Override
    public SlapdownControlConstants getControlConstants() {
        return disabledConstants;
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