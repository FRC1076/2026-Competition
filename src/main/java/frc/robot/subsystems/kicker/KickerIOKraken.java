package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import lib.units.TalonFXUnitConverter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.kicker.KickerConstants.FollowControl;
import frc.robot.subsystems.kicker.KickerConstants.LeadControl;

public class KickerIOKraken implements KickerIO {
    private final TalonFX m_leadMotor;
    private final TalonFX m_followMotor;

    private final TalonFXConfiguration m_leadMotorConfig;
    private final TalonFXConfiguration m_followMotorConfig;

    private final TalonFXUnitConverter m_unitConverter;

    // Control signals
    private final VelocityTorqueCurrentFOC m_leadVelocityRequest = new VelocityTorqueCurrentFOC(0);
    private final VelocityTorqueCurrentFOC m_followVelocityRequest = new VelocityTorqueCurrentFOC(0);
    private final VoltageOut m_voltageRequest = new VoltageOut(0);

    // Status Signals
    private final StatusSignal<Voltage> m_leadVoltageSignal;
    private final StatusSignal<AngularVelocity> m_leadVelocitySignal;
    private final StatusSignal<Current> m_leadCurrentSignal;
    private final StatusSignal<Voltage> m_followVoltageSignal;
    private final StatusSignal<AngularVelocity> m_followVelocitySignal;
    private final StatusSignal<Current> m_followCurrentSignal;

    public KickerIOKraken() {
        m_leadMotor = new TalonFX(KickerConstants.kLeadMotorCANId, KickerConstants.kCANBus);
        m_followMotor = new TalonFX(KickerConstants.kFollowMotorCANId, KickerConstants.kCANBus);

        m_leadMotorConfig = new TalonFXConfiguration();
        m_unitConverter = new TalonFXUnitConverter();

        // Voltage and current Configs 
        m_leadMotorConfig.Voltage.PeakForwardVoltage = 12;
        m_leadMotorConfig.Voltage.PeakReverseVoltage = -12;
        m_leadMotorConfig.CurrentLimits.SupplyCurrentLimit = KickerConstants.kSupplyCurrentLimit;
        m_leadMotorConfig.CurrentLimits.StatorCurrentLimit = KickerConstants.kStatorCurrentLimit;

        //Inverted
        m_leadMotorConfig.MotorOutput.Inverted = KickerConstants.kPositiveDirection;

        //set brake mode
        m_leadMotorConfig.MotorOutput.NeutralMode = KickerConstants.kNeutralMode;

        // Gear ratio
        m_leadMotorConfig.Feedback.SensorToMechanismRatio = KickerConstants.kGearRatio;
        
        // Closed loop
        m_leadMotorConfig.Slot0.kP = m_unitConverter.fromSIkP(LeadControl.kP);
        m_leadMotorConfig.Slot0.kI = m_unitConverter.fromSIkI(LeadControl.kI);
        m_leadMotorConfig.Slot0.kD = m_unitConverter.fromSIkD(LeadControl.kD);
        m_leadMotorConfig.Slot0.kS = m_unitConverter.fromSIkS(LeadControl.kS);
        m_leadMotorConfig.Slot0.kV = m_unitConverter.fromSIkV(LeadControl.kV);
        m_leadMotorConfig.Slot0.kA = m_unitConverter.fromSIkA(LeadControl.kA);

        m_followMotorConfig = m_leadMotorConfig.clone();

        // Closed loop
        m_followMotorConfig.Slot0.kP = m_unitConverter.fromSIkP(FollowControl.kP);
        m_followMotorConfig.Slot0.kI = m_unitConverter.fromSIkI(FollowControl.kI);
        m_followMotorConfig.Slot0.kD = m_unitConverter.fromSIkD(FollowControl.kD);
        m_followMotorConfig.Slot0.kS = m_unitConverter.fromSIkS(FollowControl.kS);
        m_followMotorConfig.Slot0.kV = m_unitConverter.fromSIkV(FollowControl.kV);
        m_followMotorConfig.Slot0.kA = m_unitConverter.fromSIkA(FollowControl.kA);


        // configure the motors
        m_leadMotor.getConfigurator().apply(m_leadMotorConfig);
        m_followMotor.getConfigurator().apply(m_followMotorConfig);

        m_voltageRequest.EnableFOC = KickerConstants.kUseFOC;

        // Set up status signals
        m_leadVoltageSignal = m_leadMotor.getMotorVoltage();
        m_leadVelocitySignal = m_leadMotor.getVelocity();
        m_leadCurrentSignal = m_leadMotor.getTorqueCurrent();
        m_followVoltageSignal = m_followMotor.getMotorVoltage();
        m_followVelocitySignal = m_followMotor.getVelocity();
        m_followCurrentSignal = m_followMotor.getTorqueCurrent();
    }

    @Override
    public void setVoltage(double volts) {
        m_leadMotor.setVoltage(volts);
        m_followMotor.setVoltage(volts * KickerConstants.kBackToFrontSpeedRatio);
    }

    @Override
    public void setVelocityRadPerSec(double velocity) {
        if(velocity != 0) {
            m_leadVelocityRequest.Velocity = m_unitConverter.fromSIVel(velocity);
            m_followVelocityRequest.Velocity = m_unitConverter.fromSIVel(velocity * 1.25);
            m_leadMotor.setControl(m_leadVelocityRequest);
            m_followMotor.setControl(m_followVelocityRequest);
        } else {
            m_leadMotor.setVoltage(0);
            m_followMotor.setVoltage(0);
        }
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        StatusSignal.refreshAll(
            m_leadVoltageSignal,
            m_leadVelocitySignal,
            m_leadCurrentSignal,

            m_followVoltageSignal,
            m_followVelocitySignal,
            m_followCurrentSignal
        );

        inputs.leadMotorAppliedVoltage = m_leadVoltageSignal.getValueAsDouble();
        inputs.leadMotorVelocityRadPerSec = m_unitConverter.toSIVel(m_leadVelocitySignal.getValueAsDouble());
        inputs.leadMotorTargetVelocityRadPerSec = m_unitConverter.toSIVel(m_leadVelocityRequest.Velocity);
        inputs.leadMotorCurrentAmps = (m_leadCurrentSignal.getValueAsDouble());

        inputs.followMotorAppliedVoltage = m_followVoltageSignal.getValueAsDouble();
        inputs.followMotorVelocityRadPerSec = m_unitConverter.toSIVel(m_followVelocitySignal.getValueAsDouble());
        inputs.followMotorTargetVelocityRadPerSec = m_unitConverter.toSIVel(m_followVelocityRequest.Velocity);
        inputs.followMotorCurrentAmps = (m_followCurrentSignal.getValueAsDouble());
    }
}

