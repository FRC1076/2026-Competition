// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import lib.units.TalonFXUnitConverter;

public class RollerIOKraken implements RollerIO {
    private final TalonFX m_leadMotor;
    private final TalonFX m_followMotor;

    private final TalonFXConfiguration m_leadMotorConfig;
    private final TalonFXConfiguration m_followMotorConfig;

    private final TalonFXUnitConverter m_unitConverter = new TalonFXUnitConverter();

    private final VoltageOut m_voltageRequest;
    private final VelocityTorqueCurrentFOC m_velocityRequest;
    private final Follower m_followerRequest;

    private final StatusSignal<Voltage> m_leadVoltageSignal;
    private final StatusSignal<Current> m_leadCurrentSignal;
    private final StatusSignal<AngularVelocity> m_leadVelocitySignal;
    private final StatusSignal<Temperature> m_leadTemperatureSignal;

    private final StatusSignal<Voltage> m_followVoltageSignal;
    private final StatusSignal<Current> m_followCurrentSignal;
    private final StatusSignal<AngularVelocity> m_followVelocitySignal;
    private final StatusSignal<Temperature> m_followTemperatureSignal;

    public RollerIOKraken() {
        m_leadMotor = new TalonFX(RollerConstants.kLeadMotorPort, RollerConstants.kCANBus);
        m_followMotor = new TalonFX(RollerConstants.kFollowMotorPort, RollerConstants.kCANBus);

        m_leadMotorConfig = new TalonFXConfiguration();

        m_leadMotorConfig.Voltage.PeakForwardVoltage = 12;
        m_leadMotorConfig.Voltage.PeakReverseVoltage = -12;
        m_leadMotorConfig.CurrentLimits.SupplyCurrentLimit = RollerConstants.kSupplyCurrentLimit;
        m_leadMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_leadMotorConfig.CurrentLimits.StatorCurrentLimit = RollerConstants.kStatorCurrentLimit;
        m_leadMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_leadMotorConfig.MotorOutput.Inverted = RollerConstants.kInverted;
        m_leadMotorConfig.MotorOutput.NeutralMode = RollerConstants.kNeutralMode;

        m_leadMotorConfig.Feedback.SensorToMechanismRatio = RollerConstants.kGearRatio;

        m_leadMotorConfig.Slot0.kP = m_unitConverter.fromSIkP(RollerConstants.kP);
        m_leadMotorConfig.Slot0.kI = m_unitConverter.fromSIkI(RollerConstants.kI);
        m_leadMotorConfig.Slot0.kD = m_unitConverter.fromSIkD(RollerConstants.kD);
        m_leadMotorConfig.Slot0.kS = m_unitConverter.fromSIkS(RollerConstants.kS);
        m_leadMotorConfig.Slot0.kV = m_unitConverter.fromSIkV(RollerConstants.kV);
        m_leadMotorConfig.Slot0.kA = m_unitConverter.fromSIkA(RollerConstants.kA);

        m_leadMotorConfig.Feedback.VelocityFilterTimeConstant = RollerConstants.kVelocityFilterTimeConstant;
        
        m_followMotorConfig = m_leadMotorConfig.clone();

        m_leadMotor.getConfigurator().apply(m_leadMotorConfig);
        m_followMotor.getConfigurator().apply(m_followMotorConfig);

        m_voltageRequest = new VoltageOut(0).withEnableFOC(RollerConstants.kUseFOC);
        m_velocityRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);
        m_followerRequest = new Follower(RollerConstants.kLeadMotorPort, RollerConstants.kFollowerAlignment);

        m_followMotor.setControl(m_followerRequest);

        m_leadVoltageSignal = m_leadMotor.getMotorVoltage();
        m_leadCurrentSignal = m_leadMotor.getTorqueCurrent();
        m_leadVelocitySignal = m_leadMotor.getVelocity();
        m_leadTemperatureSignal = m_leadMotor.getDeviceTemp();

        m_followVoltageSignal = m_followMotor.getMotorVoltage();
        m_followCurrentSignal = m_followMotor.getTorqueCurrent();
        m_followVelocitySignal = m_followMotor.getVelocity();
        m_followTemperatureSignal = m_followMotor.getDeviceTemp();
    }

    /** Sets the voltage for roller's motor */
    @Override
    public void setVoltage(double volts) {
        m_voltageRequest.Output = volts;
        m_leadMotor.setControl(m_voltageRequest);
    }

    /** Sets the velocity of the rollers by FOC PID */
    @Override
    public void setVelocity(double radPerSec) {
        if (radPerSec == 0) {
            m_leadMotor.setVoltage(0);
        } else {
            m_velocityRequest.Velocity = m_unitConverter.fromSIVel(radPerSec);
            m_leadMotor.setControl(m_velocityRequest);
        }
    }

    
    @Override
    public void updateInputs(RollerIOInputs inputs) {
        StatusSignal.refreshAll(
            m_leadVoltageSignal,
            m_leadCurrentSignal,
            m_leadVelocitySignal,
            m_leadTemperatureSignal,

            m_followVoltageSignal,
            m_followCurrentSignal,
            m_followVelocitySignal,
            m_followTemperatureSignal
        );

        inputs.leadMotorAppliedVoltage = m_leadVoltageSignal.getValue().in(Volts);
        inputs.leadMotorCurrentAmps = m_leadCurrentSignal.getValueAsDouble();
        inputs.leadMotorVelocityRadPerSec = m_unitConverter.toSIVel(m_leadVelocitySignal.getValueAsDouble());
        inputs.leadMotorTempDegC = m_leadTemperatureSignal.getValueAsDouble();

        inputs.followMotorAppliedVoltage = m_followVoltageSignal.getValue().in(Volts);
        inputs.followMotorCurrentAmps = m_followCurrentSignal.getValueAsDouble();
        inputs.followMotorVelocityRadPerSec = m_unitConverter.toSIVel(m_followVelocitySignal.getValueAsDouble());
        inputs.followMotorTempDegC = m_followTemperatureSignal.getValueAsDouble();
    }
}
