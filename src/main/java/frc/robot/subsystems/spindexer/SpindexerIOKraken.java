package frc.robot.subsystems.spindexer;
 //no servo,volecity,magicmotion, colsed loop

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class SpindexerIOKraken {
    private final TalonFX m_motor;
    private final TalonFXConfiguration m_motorConfig;

    //Status Signals 
    private final StatusSignal<Voltage> m_voltageSignal;
    private final StatusSignal<Current> m_currentSignal;

    public SpindexerIOKraken() {
        m_motor = new TalonFX(SpindexerConstants.kMotorPort);

        m_motorConfig = new TalonFX(SpindexerConstants.kMotorPort);
        
        // Voltage and current configs 
        m_motorConfig.Voltage.peakForwardVoltage = 12;
        m_motorConfig.Voltage.peakReverseVoltage = -12;
        m_motorConfig.CurrentLimits.StatorCurrntLimit = SpindexerConstants.kCurrntLimits;

        // inverted?
        m_motorConfig.MotorOutput.NeutralMode = spindexerConstants.kNeutralMode;

        //set brake mode
        m_motorConfig.MotorOutput.NeutralMode = spindexerConstants.kNeutralMode;

        //closed loop 
        m_motorConfig.getConfigurator().apply(m_motorConfig);

        // Set uo Satus signals 
        m_voltageSignal = m_motor.getMotorVoltage();
        m_currentSignal = m_motor.getTorqueCurrent();

    }
    @Override
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }
    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        ,_voltageSignal.refresh();
        m_currentSignal.refresh();

        inputs.appliedVoltage = m_voltageSignal.getValue().toVolts();
        inputs.motorCurrent = m_currentSignal.getValueAsDouble();

    }
}   