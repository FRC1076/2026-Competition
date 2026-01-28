package frc.robot.subsystems.spindexer;
 //no servo,volecity,magicmotion, colsed loop

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class SpindexerIOKraken implements SpindexerIO {
    private final TalonFX m_motor;
    private final TalonFXConfiguration m_motorConfig;

    //Status Signals 
    private final StatusSignal<Voltage> m_voltageSignal;
    private final StatusSignal<Current> m_currentSignal;

    public SpindexerIOKraken() {
        m_motor = new TalonFX(SpindexerConstants.kMotorPort);

        m_motorConfig = new TalonFXConfiguration();
        
        // Voltage and current configs 
        m_motorConfig.Voltage.PeakForwardVoltage = 12;
        m_motorConfig.Voltage.PeakReverseVoltage = -12;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = SpindexerConstants.kCurrentLimit;

        // inverted?
        m_motorConfig.MotorOutput.Inverted = SpindexerConstants.kInverted;

        //set brake mode
        m_motorConfig.MotorOutput.NeutralMode = SpindexerConstants.kNeutralMode;

        //closed loop 
        m_motor.getConfigurator().apply(m_motorConfig);

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
        m_voltageSignal.refresh();
        m_currentSignal.refresh();

        inputs.appliedVoltage = m_voltageSignal.getValue().in(Volts);
        inputs.currentAmps = m_currentSignal.getValueAsDouble();
    }
}   