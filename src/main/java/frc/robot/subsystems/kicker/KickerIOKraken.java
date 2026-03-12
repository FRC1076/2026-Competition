package frc.robot.subsystems.kicker;

import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.kicker.KickerConstants.Control;


import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


import lib.units.TalonFXUnitConverter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class KickerIOKraken implements KickerIO {
    private TalonFX m_motor;
    private TalonFXConfiguration m_motorConfig;
    private TalonFXUnitConverter m_unitConverter;

    // velocity control 
    private final MotionMagicVelocityVoltage m_velocityRequest = new MotionMagicVelocityVoltage(0);

    // Status Signals
    private StatusSignal<Voltage> m_voltageSignal;
    private StatusSignal<AngularVelocity> m_velocitySignal;
    private StatusSignal<Current> m_currentSignal;


    public void KickerIOTalon() {
        m_motor = new TalonFX(KickerConstants.kMotorPort);

        m_motorConfig = new TalonFXConfiguration();
        m_unitConverter = new TalonFXUnitConverter();

        // Voltage and current Configs 
        m_motorConfig.Voltage.PeakForwardVoltage = 12;
        m_motorConfig.Voltage.PeakReverseVoltage = -12;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = KickerConstants.kCurrentLimitAmps;

        //Inverted
        m_motorConfig.MotorOutput.Inverted = KickerConstants.kInverted;

        //set brale mode
        m_motorConfig.MotorOutput.NeutralMode = KickerConstants.kIdleMode;
        
        // Closed loop
        m_motorConfig.Slot0.kP = m_unitConverter.fromSIkp(Control.kP);
        m_motorConfig.Slot0.kI = m_unitConverter.fromSIkp(Control.kI);
        m_motorConfig.Slot0.kD = m_unitConverter.fromSIkp(Control.kD);
        m_motorConfig.Slot0.kS = m_unitConverter.fromSIkp(Control.kS);
        m_motorConfig.Slot0.kV = m_unitConverter.fromSIkp(Control.kV);
        m_motorConfig.Slot0.kA = m_unitConverter.fromSIkp(Control.kA);
        
       // m_motorConfig.MotionMagic.MotionMagicAcceleration = m_unitConverter.fromSIAccel(Control.kMaxAcceleration);
       // m_motorConfig.MotionMagic.MotionMagicJerk = m_unitConverter.fromSIJerk(Control.kMaxJerk);

        // config the motor
        m_motor.getConfigurator().apply(m_motorConfig);

        // Set up status signals
        m_voltageSignal = m_motor.getMotorVoltage();
        m_velocitySignal = m_motor.getVelocity();
        m_currentSignal = m_motor.getTorqueCurrent();
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setVoltage((volts));
    }

    @Override
    public void SetVelocityRadPerSec(double velocity) {
        if(velocity != 0) {
            m_velocityRequest.Velocity = m_unitConverter.fromSIVel(velocity);
            m_motor.setControl(m_velocityRequest);
        } else {
            m_motor.setVoltage(0);
        }
    
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        m_voltageSignal.refresh();
        m_velocitySignal.refresh();

        inputs.appliedVoltage = m_voltageSignal.getValueAsDouble();
        inputs.currentAmps = m_unitConverter.toSIVel(m_velocitySignal.getValueAsDouble());
    }
}

