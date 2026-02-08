package frc.robot.subsystems.slapdown;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import lib.units.TalonFXUnitConverter;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class SlapdownIOKraken implements SlapdownIO {
    private final TalonFX m_motor;
    private final TalonFXConfiguration m_motorConfig;

    private final StatusSignal<Voltage> m_voltageSignal;
    private final StatusSignal<Current> m_currentSignal;
    private final StatusSignal<AngularVelocity> m_velocitySignal;
    private final StatusSignal<Temperature> m_temperatureSignal;
    private final StatusSignal<Angle> m_positionSignal;

    private final TalonFXUnitConverter m_unitConverter;

    public SlapdownIOKraken() {
        m_motor = new TalonFX(SlapdownConstants.k_MotorPort, SlapdownConstants.kCANBus);

        m_motorConfig = new TalonFXConfiguration();
        m_unitConverter = new TalonFXUnitConverter();


        m_motorConfig.Voltage.PeakForwardVoltage = 12;
        m_motorConfig.Voltage.PeakReverseVoltage = -12;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit = SlapdownConstants.kSupplyCurrentLimit;
        m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = SlapdownConstants.kStatorCurrentLimit;
        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        m_motorConfig.MotorOutput.Inverted = SlapdownConstants.kInverted;
        m_motorConfig.MotorOutput.NeutralMode = SlapdownConstants.kNeutralMode;

        m_motor.getConfigurator().apply(m_motorConfig);

        m_voltageSignal = m_motor.getMotorVoltage();
        m_currentSignal = m_motor.getTorqueCurrent();
        m_velocitySignal = m_motor.getVelocity();
        m_temperatureSignal = m_motor.getDeviceTemp();
        m_positionSignal = m_motor.getPosition();
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }

    @Override
    public void setPosition(double positionRadians) {
        m_motor.setPosition(positionRadians / SlapdownConstants.kPositionConversionFactor);
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

        m_motorConfig.MotionMagic.MotionMagicCruiseVelocity = m_unitConverter.fromSIVel(SlapdownConstants.kCruiseVelocityRadPerSec);
        m_motorConfig.MotionMagic.MotionMagicAcceleration = m_unitConverter.fromSIAccel(SlapdownConstants.kMaxAccelRadPerSec2);
        m_motorConfig.MotionMagic.MotionMagicJerk = m_unitConverter.fromSIJerk(SlapdownConstants.kMaxJerkRadPerSec3);

        inputs.appliedVoltage = m_voltageSignal.getValue().in(Volts);
        inputs.currentAmps = m_currentSignal.getValueAsDouble();
        inputs.velocityRadiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(m_velocitySignal.getValueAsDouble());
        inputs.motorTempDegC = m_temperatureSignal.getValueAsDouble();
        inputs.position = m_positionSignal.getValueAsDouble();
    }
}
