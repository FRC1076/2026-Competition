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

public class KickerIOKraken implements KickerIO {
    private final TalonFX m_leadMotor;
    private final TalonFX m_followMotor;

    private final TalonFXConfiguration m_leadMotorConfig;
    private final TalonFXConfiguration m_followMotorConfig;

    private final TalonFXUnitConverter m_unitConverter;

    // Control signals
    private final VelocityTorqueCurrentFOC m_velocityRequest = new VelocityTorqueCurrentFOC(0);
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
        m_leadMotorConfig.Slot0.kP = m_unitConverter.fromSIkP(KickerConstants.kP);
        m_leadMotorConfig.Slot0.kI = m_unitConverter.fromSIkI(KickerConstants.kI);
        m_leadMotorConfig.Slot0.kD = m_unitConverter.fromSIkD(KickerConstants.kD);
        m_leadMotorConfig.Slot0.kS = m_unitConverter.fromSIkS(KickerConstants.kS);
        m_leadMotorConfig.Slot0.kV = m_unitConverter.fromSIkV(KickerConstants.kV);
        m_leadMotorConfig.Slot0.kA = m_unitConverter.fromSIkA(KickerConstants.kA);

        m_followMotorConfig = m_leadMotorConfig.clone();

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
        m_leadMotor.setVoltage((volts));
    }

    @Override
    public void setVelocityRadPerSec(double velocity) {
        if(velocity != 0) {
            m_velocityRequest.Velocity = m_unitConverter.fromSIVel(velocity);
            m_leadMotor.setControl(m_velocityRequest);
        } else {
            m_leadMotor.setVoltage(0);
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
        inputs.leadMotorCurrentAmps = (m_leadCurrentSignal.getValueAsDouble());

        inputs.followMotorAppliedVoltage = m_followVoltageSignal.getValueAsDouble();
        inputs.followMotorVelocityRadPerSec = m_unitConverter.toSIVel(m_followVelocitySignal.getValueAsDouble());
        inputs.followMotorCurrentAmps = (m_followCurrentSignal.getValueAsDouble());
    }
}

