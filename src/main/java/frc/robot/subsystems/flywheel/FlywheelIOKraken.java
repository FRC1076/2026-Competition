package frc.robot.subsystems.flywheel;

import lib.units.TalonFXUnitConverter;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;

public class FlywheelIOKraken implements FlywheelIO {
    // Create a motor, configuration, and unit converter here
    private final TalonFX m_motor;
    private final TalonFXConfiguration m_motorConfig;
    private final TalonFXUnitConverter m_unitConverter;

    // Make a control requests here
    private final VoltageOut m_voltageRequest;
    // private final MotionMagicVelocityVoltage m_velocityRequest;
    private final MotionMagicVelocityTorqueCurrentFOC m_velocityRequest;

    // Make voltage, velocity, current, and temperature status signals here
    private final StatusSignal<Voltage> m_voltageSignal;
    private final StatusSignal<AngularVelocity> m_velocitySignal;
    private final StatusSignal<Current> m_currentSignal;
    private final StatusSignal<Temperature> m_temperatureSignal;

    public FlywheelIOKraken() {
        // Instantiate motor here
        m_motor = new TalonFX(FlywheelConstants.kCANId, FlywheelConstants.kCANBus);

        // Instantiate configuration and unit converter
        m_unitConverter = new TalonFXUnitConverter();
        m_motorConfig = new TalonFXConfiguration();

        // Configure voltage and current limits
        m_motorConfig.Voltage.PeakForwardVoltage = FlywheelConstants.kMaxVoltage;
        m_motorConfig.Voltage.PeakReverseVoltage = -1 * FlywheelConstants.kMaxVoltage;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = FlywheelConstants.kStatorCurrentLimit;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit = FlywheelConstants.kSupplyCurrentLimit;

        // Set inverted based on constants
        m_motorConfig.MotorOutput.Inverted = FlywheelConstants.kInverted;

        // Set brake mode based on constants
        m_motorConfig.MotorOutput.NeutralMode = FlywheelConstants.kNeutralMode;

        // Configure motiom magic
        m_motorConfig.Slot0.kP = m_unitConverter.fromSIkP(FlywheelConstants.kP);
        m_motorConfig.Slot0.kI = m_unitConverter.fromSIkI(FlywheelConstants.kI);
        m_motorConfig.Slot0.kD = m_unitConverter.fromSIkD(FlywheelConstants.kD);
        m_motorConfig.Slot0.kS = m_unitConverter.fromSIkS(FlywheelConstants.kS);
        m_motorConfig.Slot0.kV = m_unitConverter.fromSIkV(FlywheelConstants.kV);
        m_motorConfig.Slot0.kA = m_unitConverter.fromSIkA(FlywheelConstants.kA);

        // Apply the configuration to the motor
        m_motor.getConfigurator().apply(m_motorConfig);

        // Set up status signals
        m_voltageSignal = m_motor.getMotorVoltage();
        m_velocitySignal = m_motor.getVelocity();
        m_currentSignal = m_motor.getTorqueCurrent();
        m_temperatureSignal = m_motor.getDeviceTemp();

        m_voltageRequest = new VoltageOut(0)
            .withEnableFOC(FlywheelConstants.kEnableFOC)
            .withOverrideBrakeDurNeutral(true);
        m_velocityRequest = new MotionMagicVelocityTorqueCurrentFOC(0);
    }

    @Override
    public void setVoltage(double volts) {
        // Set the voltage of the motor
        m_voltageRequest.withOutput(volts);
        m_motor.setControl(m_voltageRequest);
    }

    @Override
    public void setVelocityRadPerSec(double velocityRadPerSec) {
        // Set the velocity of the motor
        if (velocityRadPerSec != 0) {
            m_velocityRequest.Velocity = m_unitConverter.fromSIVel(velocityRadPerSec);
            m_motor.setControl(m_velocityRequest);
        } else {
            m_motor.setVoltage(0);
        }
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        // Update inputs based on status signals
        StatusSignal.refreshAll(
            m_voltageSignal,
            m_velocitySignal,
            m_currentSignal,
            m_temperatureSignal
        );

        inputs.appliedVoltage = m_voltageSignal.getValueAsDouble();
        inputs.velocityRadiansPerSecond = m_unitConverter.toSIVel(m_velocitySignal.getValueAsDouble());
        inputs.currentAmps = m_currentSignal.getValueAsDouble();
        inputs.temperatureDegC = m_temperatureSignal.getValueAsDouble();

        Logger.recordOutput("Flywheel/MagicMotionVelocitySetpoint", m_velocityRequest.getVelocityMeasure().in(RadiansPerSecond));
    }
}