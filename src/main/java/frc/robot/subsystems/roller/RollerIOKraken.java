// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class RollerIOKraken implements RollerIO {
    private final TalonFX m_motor;
    private final TalonFXConfiguration m_motorConfig;

    private final StatusSignal<Voltage> m_voltageSignal;
    private final StatusSignal<Current> m_currentSignal;
    private final StatusSignal<AngularVelocity> m_velocitySignal;
    private final StatusSignal<Temperature> m_temperatureSignal;

    public RollerIOKraken() {
        m_motor = new TalonFX(RollerConstants.kMotorPort, RollerConstants.kCANBus);

        m_motorConfig = new TalonFXConfiguration();

        m_motorConfig.Voltage.PeakForwardVoltage = 12;
        m_motorConfig.Voltage.PeakReverseVoltage = -12;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit = RollerConstants.kSupplyCurrentLimit;
        m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = RollerConstants.kStatorCurrentLimit;
        m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_motorConfig.MotorOutput.Inverted = RollerConstants.kInverted;
        m_motorConfig.MotorOutput.NeutralMode = RollerConstants.kNeutralMode;

        m_motorConfig.Feedback.SensorToMechanismRatio = RollerConstants.kGearRatio;

        m_motor.getConfigurator().apply(m_motorConfig);

        m_voltageSignal = m_motor.getMotorVoltage();
        m_currentSignal = m_motor.getTorqueCurrent();
        m_velocitySignal = m_motor.getVelocity();
        m_temperatureSignal = m_motor.getDeviceTemp();
    }

    /** Sets the voltage for roller's motor */
    @Override
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }

    
    @Override
    public void updateInputs(RollerIOInputs inputs) {
        StatusSignal.refreshAll(
            m_voltageSignal,
            m_currentSignal,
            m_temperatureSignal,
            m_velocitySignal
        );

        inputs.appliedVoltage = m_voltageSignal.getValue().in(Volts);
        inputs.currentAmps = m_currentSignal.getValueAsDouble();
        inputs.velocityRadPerSec = m_velocitySignal.getValueAsDouble() * Math.PI * 2;
        inputs.motorTempDegC = m_temperatureSignal.getValueAsDouble();
    }
}
