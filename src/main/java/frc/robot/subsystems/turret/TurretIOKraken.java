// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import lib.hardware.BeamBreak;
import lib.units.TalonFXUnitConverter;

public class TurretIOKraken implements TurretIO {
    private final TalonFX m_motor;
    private final CANcoder m_encoder;

    private final TalonFXConfiguration m_motorConfig;
    private final TalonFXUnitConverter m_unitConverter;

    // Beam break to rezero turret
    private final BeamBreak m_rezeroingBeamBreak;

    // Status signals
    private final StatusSignal<Voltage> m_voltageSignal;
    private final StatusSignal<Current> m_currentSignal;
    private final StatusSignal<Angle> m_motorPositionSignal;
    private final StatusSignal<Angle> m_rotorPositionSignal;
    private final StatusSignal<AngularVelocity> m_velocitySignal;
    private final StatusSignal<Temperature> m_temperatureSignal;
    private final BooleanSupplier m_encoderResetSignal;

    // Control requests
    private final VoltageOut m_voltageRequest;
    private final VoltageOut m_voltageRequestNoSoftStops;
    private final MotionMagicVoltage m_positionRequest;

    public TurretIOKraken() {
        m_motor = new TalonFX(TurretConstants.kCANId, TurretConstants.kCANBus);
        m_encoder = new CANcoder(TurretConstants.kCANcoderCANId, TurretConstants.kCANBus);
        m_motorConfig = new TalonFXConfiguration();
        m_unitConverter = new TalonFXUnitConverter();

        m_rezeroingBeamBreak = new BeamBreak(TurretConstants.kBeamBreakPort);

        // Voltage and current configs
        m_motorConfig.Voltage.PeakForwardVoltage = TurretConstants.kMaxVoltage;
        m_motorConfig.Voltage.PeakReverseVoltage = -1 * TurretConstants.kMaxVoltage;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = TurretConstants.kStatorCurrentLimitAmps;
        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit = TurretConstants.kSupplyCurrentLimitAmps;
        m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Inverted?
        m_motorConfig.MotorOutput.Inverted = TurretConstants.kInvertedValue;

        // Set brake mode
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // CANcoder
        m_motorConfig.Feedback.FeedbackRemoteSensorID = TurretConstants.kCANcoderCANId;
        m_motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder; // TODO: Consider using FusedCANcoder or Sync
        m_motorConfig.Feedback.RotorToSensorRatio = TurretConstants.kRotorToSensorRatio;

        // Offset from internal absolute encoder
        // m_motorConfig.Feedback.FeedbackRotorOffset = TurretConstants.kEncoderOffsetRot; // We'll just start at zero
        m_motorConfig.Feedback.SensorToMechanismRatio = TurretConstants.kSensorToMechanismRatio;

        // Closed loop
        m_motorConfig.Slot0.kP = m_unitConverter.fromSIkP(TurretConstants.kP);
        m_motorConfig.Slot0.kI = m_unitConverter.fromSIkI(TurretConstants.kI);
        m_motorConfig.Slot0.kD = m_unitConverter.fromSIkD(TurretConstants.kD);
        m_motorConfig.Slot0.kS = m_unitConverter.fromSIkS(TurretConstants.kS);
        m_motorConfig.Slot0.kV = m_unitConverter.fromSIkV(TurretConstants.kV);
        m_motorConfig.Slot0.kA = m_unitConverter.fromSIkA(TurretConstants.kA);

        // Motion magic
        m_motorConfig.MotionMagic.MotionMagicCruiseVelocity = m_unitConverter.fromSIVel(TurretConstants.kCruiseVelocityRadPerSec);
        m_motorConfig.MotionMagic.MotionMagicAcceleration = m_unitConverter.fromSIAccel(TurretConstants.kMaxAccelRadPerSec2);
        m_motorConfig.MotionMagic.MotionMagicJerk = m_unitConverter.fromSIJerk(TurretConstants.kMaxJerkRadPerSec3);

        // Software Stops
        m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = m_unitConverter.fromSIPos(TurretConstants.kMaxPositionRad);
        m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = m_unitConverter.fromSIPos(TurretConstants.kMinPositionRad);
        m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        // Apply configs
        m_motor.getConfigurator().apply(m_motorConfig);
        m_motor.setPosition(0);
        m_encoder.setPosition(0); // Start pointing forward

        // Status signals
        m_voltageSignal = m_motor.getMotorVoltage();
        m_currentSignal = m_motor.getTorqueCurrent();
        m_motorPositionSignal = m_motor.getPosition();
        m_rotorPositionSignal = m_motor.getRotorPosition();
        m_velocitySignal = m_motor.getVelocity();
        m_temperatureSignal = m_motor.getDeviceTemp();
        m_encoderResetSignal = m_encoder.getResetOccurredChecker();

        // Set up control requests
        m_voltageRequest = new VoltageOut(0)
            .withEnableFOC(TurretConstants.kEnableFOC);
        m_voltageRequestNoSoftStops = new VoltageOut(0)
            .withEnableFOC(TurretConstants.kEnableFOC)
            .withIgnoreSoftwareLimits(true);
        m_positionRequest = new MotionMagicVoltage(0)
            .withSlot(0)
            .withEnableFOC(TurretConstants.kEnableFOC);
    }

    /** Sets the voltage for the turret's motor */
    @Override
    public void setVoltage(double volts) {
        m_voltageRequest.Output = volts;
        m_motor.setControl(m_voltageRequest);
    }

    /** Sets the voltage of the turret's motor, ignoring software stops */
    @Override
    public void setVoltageNoSoftStops(double volts) {
        m_voltageRequestNoSoftStops.Output = volts;
        m_motor.setControl(m_voltageRequestNoSoftStops);
    }

    /**Sets the target position of the turret motor */
    @Override
    public void setPosition(double positionRadians) {
        m_positionRequest.Position = m_unitConverter.fromSIPos(positionRadians);
        m_motor.setControl(m_positionRequest);
    }

    @Override
    public void resetPosition() {
        m_motor.setPosition(0);
        m_encoder.setPosition(0);
    }

    @Override
    public void resetPositionTo(double position) {
        m_motor.setPosition(m_unitConverter.fromSIPos(position));
        m_encoder.setPosition(m_unitConverter.fromSIPos(position));
    }

    @Override
    public void periodic() {
        // Rezero the turret if it passes by the beam break
        if (m_rezeroingBeamBreak.isBeamBroken()) {
            // TODO: uncomment this if we want to test it
            //m_encoder.setPosition(TurretConstants.kBeamBreakRezeroingPosition, 0.01);
        }
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        StatusSignal.refreshAll(
            m_voltageSignal,
            m_currentSignal,
            m_motorPositionSignal,
            m_rotorPositionSignal,
            m_velocitySignal,
            m_temperatureSignal
        );
        
        inputs.motorAppliedVoltage = m_voltageSignal.getValueAsDouble();
        inputs.motorCurrentAmps = m_currentSignal.getValueAsDouble();
        inputs.motorPositionRad = m_unitConverter.toSIPos(m_motorPositionSignal.getValueAsDouble());
        inputs.rotorPositionRad = m_unitConverter.toSIPos(m_rotorPositionSignal.getValueAsDouble());
        inputs.motorVelocityRadPerSec = m_unitConverter.toSIVel(m_velocitySignal.getValueAsDouble());
        inputs.motorTempDegC = m_temperatureSignal.getValueAsDouble();
        inputs.beamBroken = m_rezeroingBeamBreak.isBeamBroken();
        inputs.hasReset = m_encoderResetSignal.getAsBoolean();

        Logger.recordOutput("Turret/PositionTargetRad", m_positionRequest.getPositionMeasure().in(Radians));
    }
}