package frc.robot.subsystems.slapdown;

public class SlapdownIODisabled implements SlapdownIO {
    private double voltageTarget = 0;
    private double positionTarget = 0;

    @Override
    public void setVoltage(double volts) {
        voltageTarget = volts;
    }

    @Override
    public void setPosition(double radians) {
        positionTarget = radians;
        
    }

    @Override
    public void updateInputs(SlapdownIOInputs inputs) {
        inputs.appliedVoltage = voltageTarget;
        inputs.angleRadians = positionTarget;
    }

    @Override
    public void stop() {
        this.setVoltage(0.0);
    }
}