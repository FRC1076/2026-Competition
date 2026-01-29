package frc.robot.subsystems.kicker;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class KickerIONeo implements KickerIO{
    private final SparkMax m_motor; 
    private final SparkMaxConfig m_motorConfig;

    public KickerIONeo() {
        m_motor = new SparkMax(KickerConstants.kCanId, MotorType.kBrushless);
        m_motorConfig = new SparkMaxConfig();

        m_motorConfig
            .voltageCompensation(KickerConstants.kVoltageCompensation)
            .smartCurrentLimit(KickerConstants.kCurrentLimitAmps);

        m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override 
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }

    @Override 
    public void updateInputs(KickerIOInputs inputs) {
        inputs.appliedVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.currentAmps = m_motor.getOutputCurrent();
    }
}
