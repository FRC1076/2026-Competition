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

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;

public class FlywheelIOKraken implements FlywheelIO {
    // Create a motor, configuration, and unit converter here
    private final TalonFX m_leadMotor;
    private final TalonFXConfiguration m_leadMotorConfig;

    private final TalonFX m_followMotor;
    private final TalonFXConfiguration m_followMotorConfig;
    
    private final TalonFXUnitConverter m_unitConverter;


    // Make a control requests here
    private final VoltageOut m_voltageRequest;
    // private final MotionMagicVelocityVoltage m_velocityRequest;
    private final MotionMagicVelocityTorqueCurrentFOC m_velocityRequest;

    // Make voltage, velocity, current, and temperature status signals here
    private final StatusSignal<Voltage> m_leadVoltageSignal;
    private final StatusSignal<Current> m_leadCurrentSignal;
    private final StatusSignal<Temperature> m_leadTemperatureSignal;

    private final StatusSignal<Voltage> m_followVoltageSignal;
    private final StatusSignal<Current> m_followCurrentSignal;
    private final StatusSignal<Temperature> m_followTemperatureSignal;

    private final StatusSignal<AngularVelocity> m_velocitySignal;

    public FlywheelIOKraken() {
        // Instantiate motor here
        m_leadMotor = new TalonFX(FlywheelConstants.kLeadMotorCANId, FlywheelConstants.kCANBus);
        m_followMotor = new TalonFX(FlywheelConstants.kFollowMotorCANId, FlywheelConstants.kCANBus);

        // Instantiate configuration and unit converter
        m_unitConverter = new TalonFXUnitConverter();
        m_leadMotorConfig = new TalonFXConfiguration();

        // Configure voltage and current limits
        m_leadMotorConfig.Voltage.PeakForwardVoltage = FlywheelConstants.kMaxVoltage;
        m_leadMotorConfig.Voltage.PeakReverseVoltage = -1 * FlywheelConstants.kMaxVoltage;
        m_leadMotorConfig.CurrentLimits.StatorCurrentLimit = FlywheelConstants.kStatorCurrentLimit;
        m_leadMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        m_leadMotorConfig.CurrentLimits.SupplyCurrentLimit = FlywheelConstants.kSupplyCurrentLimit;
        m_leadMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Set inverted based on constants
        m_leadMotorConfig.MotorOutput.Inverted = FlywheelConstants.kInverted;

        // Set brake mode based on constants
        m_leadMotorConfig.MotorOutput.NeutralMode = FlywheelConstants.kNeutralMode;

        // Configure motiom magic
        m_leadMotorConfig.Slot0.kP = m_unitConverter.fromSIkP(FlywheelConstants.kP);
        m_leadMotorConfig.Slot0.kI = m_unitConverter.fromSIkI(FlywheelConstants.kI);
        m_leadMotorConfig.Slot0.kD = m_unitConverter.fromSIkD(FlywheelConstants.kD);
        m_leadMotorConfig.Slot0.kS = m_unitConverter.fromSIkS(FlywheelConstants.kS);
        m_leadMotorConfig.Slot0.kV = m_unitConverter.fromSIkV(FlywheelConstants.kV);
        m_leadMotorConfig.Slot0.kA = m_unitConverter.fromSIkA(FlywheelConstants.kA);

        m_followMotorConfig = m_leadMotorConfig.clone();
        m_followMotor.setControl(new Follower(FlywheelConstants.kLeadMotorCANId, null));
        

        // Apply the configuration to the motor
        m_leadMotor.getConfigurator().apply(m_leadMotorConfig);
        m_followMotor.getConfigurator().apply(m_followMotorConfig);

        // Set up status signals
        m_leadVoltageSignal = m_leadMotor.getMotorVoltage();
        m_leadCurrentSignal = m_leadMotor.getTorqueCurrent();
        m_leadTemperatureSignal = m_leadMotor.getDeviceTemp();

        m_followVoltageSignal = m_followMotor.getMotorVoltage();
        m_followCurrentSignal = m_followMotor.getTorqueCurrent();
        m_followTemperatureSignal = m_followMotor.getDeviceTemp();

        m_velocitySignal = m_leadMotor.getVelocity();

        m_voltageRequest = new VoltageOut(0)
            .withEnableFOC(FlywheelConstants.kEnableFOC)
            .withOverrideBrakeDurNeutral(true);
        m_velocityRequest = new MotionMagicVelocityTorqueCurrentFOC(0);
    }

    @Override
    public void setVoltage(double volts) {
        // Set the voltage of the motor
        m_voltageRequest.withOutput(volts);
        m_leadMotor.setControl(m_voltageRequest);
    }

    @Override
    public void setVelocityRadPerSec(double velocityRadPerSec) {
        // Set the velocity of the motor
        if (velocityRadPerSec != 0) {
            m_velocityRequest.Velocity = m_unitConverter.fromSIVel(velocityRadPerSec);
            m_leadMotor.setControl(m_velocityRequest);
        } else {
            m_leadMotor.setVoltage(0);
        }
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        // Update inputs based on status signals
        StatusSignal.refreshAll(
            m_leadVoltageSignal,
            m_leadCurrentSignal,
            m_leadTemperatureSignal,

            m_followVoltageSignal,
            m_followCurrentSignal,
            m_followTemperatureSignal,

            m_velocitySignal
        );

        inputs.appliedLeadVoltage = m_leadVoltageSignal.getValueAsDouble();
        inputs.leadCurrentAmps = m_leadCurrentSignal.getValueAsDouble();
        inputs.leadTemperatureDegC = m_leadTemperatureSignal.getValueAsDouble();

        inputs.followAppliedVoltage = m_followVoltageSignal.getValueAsDouble();
        inputs.followCurrentAmps = m_followCurrentSignal.getValueAsDouble();
        inputs.followTemperatureDegC = m_followTemperatureSignal.getValueAsDouble();

        inputs.velocityRadiansPerSecond = m_unitConverter.toSIVel(m_velocitySignal.getValueAsDouble());


        Logger.recordOutput("Flywheel/MagicMotionVelocitySetpoint", m_velocityRequest.getVelocityMeasure().in(RadiansPerSecond));
    }
}