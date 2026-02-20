// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class SpindexerIOKraken implements SpindexerIO {
    private final TalonFX m_motor;
    private final TalonFXConfiguration m_motorConfig;

    //Status Signals 
    private final StatusSignal<Voltage> m_voltageSignal;
    private final StatusSignal<Current> m_currentSignal;
    private final StatusSignal<AngularVelocity> m_velocitySignal;
    private final StatusSignal<Temperature> m_temperatureSignal;

    public SpindexerIOKraken() {
        m_motor = new TalonFX(SpindexerConstants.kCANId, SpindexerConstants.kCANBus);

        m_motorConfig = new TalonFXConfiguration();
        
        // Voltage and current configs 
        m_motorConfig.Voltage.PeakForwardVoltage = 12;
        m_motorConfig.Voltage.PeakReverseVoltage = -12;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit = SpindexerConstants.kSupplyCurrentLimit;
        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = SpindexerConstants.kStatorCurrentLimit;
        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // Inverted
        m_motorConfig.MotorOutput.Inverted = SpindexerConstants.kInverted;

        // Set brake mode
        m_motorConfig.MotorOutput.NeutralMode = SpindexerConstants.kNeutralMode;

        // Gear ratio
        m_motorConfig.Feedback.SensorToMechanismRatio = SpindexerConstants.kGearRatio;

        //closed loop 
        m_motor.getConfigurator().apply(m_motorConfig);

        // Set uo Satus signals 
        m_voltageSignal = m_motor.getMotorVoltage();
        m_currentSignal = m_motor.getTorqueCurrent();
        m_velocitySignal = m_motor.getVelocity();
        m_temperatureSignal = m_motor.getDeviceTemp();

    }

    /**Sets the voltage for the Spindexer's motor */
    @Override
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        StatusSignal.refreshAll(
            m_voltageSignal, 
            m_currentSignal, 
            m_temperatureSignal,
            m_velocitySignal
        );

        inputs.appliedVoltage = m_voltageSignal.getValueAsDouble();
        inputs.currentAmps = m_currentSignal.getValueAsDouble();
        inputs.velocityRadPerSec = m_velocitySignal.getValueAsDouble() * 2 * Math.PI;
        inputs.motorTempDegC = m_temperatureSignal.getValueAsDouble();
    }
}   