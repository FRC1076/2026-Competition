// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.kicker;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class KickerIONeo implements KickerIO{
    private final SparkMax m_motor; 
    private final SparkMaxConfig m_motorConfig;

    private final RelativeEncoder m_encoder;

    public KickerIONeo() {
        m_motor = new SparkMax(KickerConstants.kCanId, MotorType.kBrushless);
        m_motorConfig = new SparkMaxConfig();

        m_motorConfig
            .voltageCompensation(KickerConstants.kVoltageCompensation)
            .smartCurrentLimit(KickerConstants.kCurrentLimitAmps)
            .inverted(KickerConstants.kInverted)
            .idleMode(KickerConstants.kIdleMode);

        m_motorConfig.encoder
            .velocityConversionFactor(KickerConstants.kVelocityConversionFactor);

        m_motorConfig.softLimit
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimitEnabled(false); 

        m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_encoder = m_motor.getEncoder();
    }
    /** Sets the voltage for Kicker's motor */
    @Override 
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }

    /** Apply voltage to kicker's motor */
    @Override 
    public void updateInputs(KickerIOInputs inputs) {
        inputs.appliedVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.currentAmps = m_motor.getOutputCurrent();

        inputs.velocityRadPerSec = m_encoder.getVelocity();
    }
}
