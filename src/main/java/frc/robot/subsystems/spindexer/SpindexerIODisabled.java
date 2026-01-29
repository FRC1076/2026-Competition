package frc.robot.subsystems.spindexer;

public class SpindexerIODisabled implements SpindexerIO {
    private double voltageTarget = 0.0;

    @Override
    public void setVoltage(double volts) {
        voltageTarget = volts;

    }

    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        inputs.appliedVoltage = voltageTarget;
    }
}
