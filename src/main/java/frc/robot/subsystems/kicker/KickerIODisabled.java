package frc.robot.subsystems.kicker;

public class KickerIODisabled implements KickerIO {
    private double voltageTarget = 0.0; 
    @Override
    public void setVoltage(double volts) {
        voltageTarget = volts;
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        inputs.appliedVoltage = voltageTarget;
    }
    
}
