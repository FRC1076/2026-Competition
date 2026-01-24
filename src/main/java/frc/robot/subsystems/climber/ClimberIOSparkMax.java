package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;

public class ClimberIOSparkMax implements ClimberIO {
    private final SparkMax m_motor;
    private final SparkMaxConfig m_motorConfig;

    private final RelativeEncoder m_encoder;

    private final ProfiledPIDController m_profiledPIDController;

    private boolean PIDEnabled = false;

    private double targetPosition = 0.0;

    public ClimberIOSparkMax() {

        PIDEnabled = false;

        m_profiledPIDController = new ProfiledPIDController(
            ClimberConstants.kP,
            ClimberConstants.kI,
            ClimberConstants.kD,
            ClimberConstants.kProfileConstraints
        );

        m_motor = new SparkMax(ClimberConstants.kCANId, MotorType.kBrushless);
        m_motorConfig = new SparkMaxConfig();

        m_motor.setCANTimeout(250);

        m_motorConfig
            .inverted(ClimberConstants.motorInverted)
            .smartCurrentLimit((int) ClimberConstants.kCurrentLimit)
            .voltageCompensation(ClimberConstants.kVoltageCompensation);

        m_motorConfig.encoder
            .positionConversionFactor(ClimberConstants.kPositionConversionFactor)
            .velocityConversionFactor(ClimberConstants.kVelocityConversionFactor)
            .quadratureMeasurementPeriod(10)
            .quadratureAverageDepth(2);

        
        m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        m_encoder = m_motor.getEncoder();

        m_encoder.setPosition(0);

        m_motor.setCANTimeout(0);
    }

    @Override
    public void setVoltage(double volts) {
        appliedVoltage = volts;
    }

    @Override
    public void setPosition(double positionMeters) {
        PIDEnabled = true;
        targetPosition = positionMeters;
        pidController.setReference(positionMeters, SparkMax.ControlType.kPosition);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.appliedVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.appliedOutput = m_motor.getAppliedOutput();
        inputs.currentAmps = m_motor.getOutputCurrent();

        inputs.climberPosition = m_encoder.getPosition();
        inputs.climberVelocity = m_encoder.getVelocity();
    }

    public void setPIDEnabled(boolean enabled) {
        this.PIDEnabled = enabled;
    }

    public void stop() {
        this.setPIDEnabled(false);
        this.setVoltage(0.0);
    }

}