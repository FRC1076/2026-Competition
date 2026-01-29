package frc.robot.subsystems.roller;

public class RollerIODisabled implements RollerIO {
    private double voltageTarget = 0.0;

    @Override
    public void setVoltage(double volts) {
        voltageTarget = volts;

    }
    
    @Override 
    public void updateInputs(RollerIOInputs inputs) {
        inputs.appliedVoltage = voltageTarget;
    }
}
