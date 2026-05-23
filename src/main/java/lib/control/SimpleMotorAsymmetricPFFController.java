package lib.control;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/** A controller that applies a proportional output when below the setpoint
 *  and a feedfoward all the time. Should be used for velocity only. */
public class SimpleMotorAsymmetricPFFController {
    private double kP;
    private final SimpleMotorFeedforward m_feedforward;
    private double deadband;
    private double setpoint = 0;

    public SimpleMotorAsymmetricPFFController(
        double kP,
        double kS, double kV, double kA,
        double deadband) {
        this.kP = kP;
        this.m_feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        this.deadband = deadband;
    }

    /** Calculates the output based on current and setpoint. */
    public double calculateVelocity(double current, double setpoint) {
        this.setpoint = setpoint;
        if (current > (setpoint - deadband)) {
            return m_feedforward.calculate(setpoint);
        } else {
            return m_feedforward.calculate(setpoint) + ((setpoint - current) * kP);
        }
    }

    public double calculateVelocity(double current) {
        return calculateVelocity(current, setpoint);
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public double getP() {
        return kP;
    }

    public double getS() {
        return m_feedforward.getKs();
    }

    public double getV() {
        return m_feedforward.getKv();
    }

    public double getA() {
        return m_feedforward.getKa();
    }

    public double getDeadband() {
        return deadband;
    }

    public void setP(double kP) {
        this.kP = kP;
    }

    public void setS(double kS) {
        m_feedforward.setKs(kS);
    }

    public void setV(double kV) {
        m_feedforward.setKs(kV);
    }

    public void setA(double kA) {
        m_feedforward.setKs(kA);
    }

    public void setDeadband(double deadband) {
        this.deadband = deadband;
    }
}
