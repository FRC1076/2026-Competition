public interface HoodIO {
    public static record HoodControlConstants (
        Double kP,
        Double kI,
        Double kD,
        Constraints kProfileConstraints,
        Double kS,
        Double kG,
        Double kV,
        Double kA
    ) {}

    @AutoLog
    public static class HoodIOInputs {
        public double appliedVolts = 0;
        public double leadCurrentAmps = 0;
        public double angleRadians = 0;
        public double velocityRadiansPerSecond = 0;
    }

    public abstract HoodControlConstants getControlConstants();

    public abstract void updateInputs(HoodIOInputs inputs);

    public abstract void setVoltage(double volts);
}