// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.slapdown;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import lib.units.TalonFXUnitConverter;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class SlapdownIOKraken implements SlapdownIO {
    private final TalonFX m_motor;
    private final TalonFXConfiguration m_motorConfig;

    private final VoltageOut m_voltageRequest;
    private final VoltageOut m_voltageNoSoftStopsRequest;
    private final MotionMagicVoltage m_positionRequest;

    private final StatusSignal<Voltage> m_voltageSignal;
    private final StatusSignal<Current> m_currentSignal;
    private final StatusSignal<AngularVelocity> m_velocitySignal;
    private final StatusSignal<Temperature> m_temperatureSignal;
    private final StatusSignal<Angle> m_positionSignal;

    private final TalonFXUnitConverter m_unitConverter;

    public SlapdownIOKraken() {
        m_motor = new TalonFX(SlapdownConstants.kCANId, SlapdownConstants.kCANBus);

        m_motorConfig = new TalonFXConfiguration();
        m_unitConverter = new TalonFXUnitConverter();

        m_motorConfig.Voltage.PeakForwardVoltage = 12;
        m_motorConfig.Voltage.PeakReverseVoltage = -12;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit = SlapdownConstants.kSupplyCurrentLimit;
        m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = SlapdownConstants.kStatorCurrentLimit;
        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        m_motorConfig.Feedback.SensorToMechanismRatio = SlapdownConstants.kGearRatio;
        m_motorConfig.Feedback.FeedbackRotorOffset = SlapdownConstants.kRotorOffsetRot;

        m_motorConfig.MotorOutput.Inverted = SlapdownConstants.kInverted;
        m_motorConfig.MotorOutput.NeutralMode = SlapdownConstants.kNeutralMode;

        m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = m_unitConverter.fromSIPos(SlapdownConstants.kMaxAngleRadians);
        m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = m_unitConverter.fromSIPos(SlapdownConstants.kMinAngleRadians);
        m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        m_motorConfig.Slot0.kP = m_unitConverter.fromSIkP(SlapdownConstants.kP);
        m_motorConfig.Slot0.kI = m_unitConverter.fromSIkI(SlapdownConstants.kI);
        m_motorConfig.Slot0.kD = m_unitConverter.fromSIkD(SlapdownConstants.kD);

        m_motorConfig.Slot0.kS = m_unitConverter.fromSIkS(SlapdownConstants.kS);
        m_motorConfig.Slot0.kG = m_unitConverter.fromSIkG(SlapdownConstants.kG);
        m_motorConfig.Slot0.kV = m_unitConverter.fromSIkV(SlapdownConstants.kV);
        m_motorConfig.Slot0.kA = m_unitConverter.fromSIkA(SlapdownConstants.kA);
        m_motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        m_motorConfig.MotionMagic.MotionMagicCruiseVelocity = m_unitConverter.fromSIVel(SlapdownConstants.kCruiseVelocityRadPerSec);
        m_motorConfig.MotionMagic.MotionMagicAcceleration = m_unitConverter.fromSIAccel(SlapdownConstants.kMaxAccelRadPerSec2);
        m_motorConfig.MotionMagic.MotionMagicJerk = m_unitConverter.fromSIJerk(SlapdownConstants.kMaxJerkRadPerSec3);

        m_motor.getConfigurator().apply(m_motorConfig);

        m_voltageRequest = new VoltageOut(0)
            .withEnableFOC(SlapdownConstants.kUseFOC);
        m_voltageNoSoftStopsRequest = new VoltageOut(0)
            .withEnableFOC(SlapdownConstants.kUseFOC)
            .withIgnoreSoftwareLimits(true);
        m_positionRequest = new MotionMagicVoltage(0)
            .withEnableFOC(SlapdownConstants.kUseFOC);

        m_voltageSignal = m_motor.getMotorVoltage();
        m_currentSignal = m_motor.getTorqueCurrent();
        m_velocitySignal = m_motor.getVelocity();
        m_temperatureSignal = m_motor.getDeviceTemp();
        m_positionSignal = m_motor.getPosition();
    }

    @Override
    public void setVoltage(double volts) {
        m_voltageRequest.withOutput(volts);
        m_motor.setControl(m_voltageRequest);
    }

    @Override
    public void setVoltageNoSoftStops(double volts) {
        m_voltageNoSoftStopsRequest.withOutput(volts);
        m_motor.setControl(m_voltageNoSoftStopsRequest);
    }

    @Override
    public void setPosition(double positionRadians) {
        m_positionRequest.withPosition(m_unitConverter.fromSIPos(positionRadians));
        m_motor.setControl(m_positionRequest);
    }

    @Override
    public void rezero() {
        m_motor.setPosition(0);
    }

    @Override
    public void updateInputs(SlapdownIOInputs inputs) {
        StatusSignal.refreshAll(
            m_voltageSignal,
            m_currentSignal,
            m_temperatureSignal,
            m_velocitySignal,
            m_positionSignal
        );

        inputs.appliedVoltage = m_voltageSignal.getValue().in(Volts);
        inputs.currentAmps = m_currentSignal.getValueAsDouble();
        inputs.angleRadians = m_unitConverter.toSIPos(m_positionSignal.getValueAsDouble());
        inputs.velocityRadiansPerSecond = m_unitConverter.toSIVel(m_velocitySignal.getValueAsDouble());
        inputs.motorTempDegC = m_temperatureSignal.getValueAsDouble();
    }
}
