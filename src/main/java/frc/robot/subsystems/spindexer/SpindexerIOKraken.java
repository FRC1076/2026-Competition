// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import lib.units.TalonFXUnitConverter;

public class SpindexerIOKraken implements SpindexerIO {
    private final TalonFX m_motor;
    private final TalonFXConfiguration m_motorConfig;

    private final TalonFXUnitConverter m_unitConverter;

    // Control requests
    private final VoltageOut m_voltageRequest;
    private final TorqueCurrentFOC m_currentRequest;
    private final VelocityTorqueCurrentFOC m_velocityRequest;

    //Status Signals 
    private final StatusSignal<Voltage> m_voltageSignal;
    private final StatusSignal<Current> m_currentSignal;
    private final StatusSignal<AngularVelocity> m_velocitySignal;
    private final StatusSignal<Temperature> m_temperatureSignal;

    public SpindexerIOKraken() {
        m_motor = new TalonFX(SpindexerConstants.kCANId, SpindexerConstants.kCANBus);

        m_motorConfig = new TalonFXConfiguration();

        m_unitConverter = new TalonFXUnitConverter();
        
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

        // Closed loop
        m_motorConfig.Slot0.kP = m_unitConverter.fromSIkP(SpindexerConstants.kP);
        m_motorConfig.Slot0.kI = m_unitConverter.fromSIkI(SpindexerConstants.kI);
        m_motorConfig.Slot0.kD = m_unitConverter.fromSIkD(SpindexerConstants.kD);
        m_motorConfig.Slot0.kS = m_unitConverter.fromSIkS(SpindexerConstants.kS);
        m_motorConfig.Slot0.kV = m_unitConverter.fromSIkV(SpindexerConstants.kV);
        m_motorConfig.Slot0.kA = m_unitConverter.fromSIkA(SpindexerConstants.kA);

        m_motorConfig.Feedback.VelocityFilterTimeConstant = SpindexerConstants.kVelocityFilterTimeConstant;

        m_motor.getConfigurator().apply(m_motorConfig);

        m_voltageRequest = new VoltageOut(0).withEnableFOC(SpindexerConstants.kUseFOC);
        m_currentRequest = new TorqueCurrentFOC(0);
        m_velocityRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);

        // Set up Satus signals 
        m_voltageSignal = m_motor.getMotorVoltage();
        m_currentSignal = m_motor.getTorqueCurrent();
        m_velocitySignal = m_motor.getVelocity();
        m_temperatureSignal = m_motor.getDeviceTemp();
    }

    /** Sets the voltage for spindexer's motor */
    @Override
    public void setVoltage(double volts) {
        m_voltageRequest.Output = volts;
        m_motor.setControl(m_voltageRequest);
    }

    @Override
    public void setTorque(double amps) {
        m_currentRequest.Output = amps;
        m_motor.setControl(m_currentRequest);
    }

    /** Sets the velocity of the spindexer by FOC PID */
    @Override
    public void setVelocity(double radPerSec) {
        if (radPerSec == 0) {
            m_motor.setVoltage(0);
        } else {
            m_velocityRequest.Velocity = m_unitConverter.fromSIVel(radPerSec);
            m_motor.setControl(m_velocityRequest);
        }
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
        inputs.velocityRadPerSec = m_unitConverter.toSIVel(m_velocitySignal.getValueAsDouble());
        inputs.motorTempDegC = m_temperatureSignal.getValueAsDouble();

        Logger.recordOutput("Spindexer/PIDTargetRadPerSec", m_unitConverter.toSIVel(m_velocityRequest.Velocity));
    }
}   