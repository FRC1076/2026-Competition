package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.Volts;

import java.io.ObjectInputFilter.Status;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class RollerIOKraken implements RollerIO {
    private final TalonFX m_motor;
    private final TalonFXConfiguration m_motorConfig;

    private final StatusSignal<Voltage> m_voltageSignal;
    private final StatusSignal<Current> m_currentSignal;

    public RollerIOKraken() {
        m_motor = new TalonFX(RollerConstants.kMotorPort);

        m_motorConfig = new TalonFXConfiguration();

        m_motorConfig.Voltage.PeakForwardVoltage = 12;
        m_motorConfig.Voltage.PeakReverseVoltage = -12;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = RollerConstants.kCurrentLimit;

        m_motorConfig.MotorOutput.Inverted = RollerConstants.kInverted;
        m_motorConfig.MotorOutput.NeutralMode = RollerConstants.kNeutralMode;

        m_motor.getConfigurator().apply(m_motorConfig);

        m_voltageSignal = m_motor.getMotorVoltage();
        m_currentSignal = m_motor.getTorqueCurrent();
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        m_voltageSignal.refresh();
        m_currentSignal.refresh();

        inputs.appliedVoltage = m_voltageSignal.getValue().in(Volts);
        inputs.currentAmps = m_currentSignal.getValueAsDouble();
    }
}
