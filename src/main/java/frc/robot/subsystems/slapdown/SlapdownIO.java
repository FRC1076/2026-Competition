package frc.robot.subsystems.slapdown;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public interface SlapdownIO {

    public static record SlapdownControlConstants(double kP, double kI, double kD, Constraints kProfileConstraints, double kS, double kG, double kV, double kA) {}

    @AutoLog
    public static class SlapdownIOInputs {

        public double appliedVoltage = 0;
        public double currentAmps = 0;
        public double appliedOutput = 0;

        public double motorCurrent = 0;

        public double angleRadians = 0;
        public double velocityRadiansPerSecond = 0;
    }

    public abstract SlapdownControlConstants getControlConstants();

    public abstract void setVoltage(double volts);

    public abstract void updateInputs(SlapdownIOInputs inputs);

    public abstract void stop();

}