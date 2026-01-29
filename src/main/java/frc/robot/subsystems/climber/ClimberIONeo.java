package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class ClimberIONeo implements ClimberIO {
    private final SparkMax m_motor;
    private final SparkMaxConfig m_motorConfig;

    private final RelativeEncoder m_encoder;

    private final ProfiledPIDController m_profiledPidController;
    private final ElevatorFeedforward m_feedforward;
    private boolean PIDEnabled = false;

    public ClimberIONeo() {

        m_motor = new SparkMax(ClimberConstants.kCANId, MotorType.kBrushless);
        m_motorConfig = new SparkMaxConfig();

        m_motorConfig
            .inverted(ClimberConstants.kMotorInverted)
            .smartCurrentLimit((int) ClimberConstants.kCurrentLimit)
            .voltageCompensation(ClimberConstants.kVoltageCompensation);
        
        m_motorConfig.encoder
            .positionConversionFactor(ClimberConstants.kPositionConversionFactor)
            .velocityConversionFactor(ClimberConstants.kVelocityConversionFactor)
            .quadratureMeasurementPeriod(10)
            .quadratureAverageDepth(2);

        
        m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        m_encoder = m_motor.getEncoder();

        m_encoder.setPosition(0);

        m_profiledPidController = new ProfiledPIDController(
            ClimberConstants.kP,
            ClimberConstants.kI,
            ClimberConstants.kD, 
            ClimberConstants.kProfileConstraints
        );

        m_feedforward = new ElevatorFeedforward(
            ClimberConstants.kS,
            ClimberConstants.kG,
            ClimberConstants.kV,
            ClimberConstants.kA
        );
    }

    @Override
    public void setVoltage(double volts) {
        PIDEnabled = false;
        m_motor.setVoltage(volts);
    }

    @Override
    public void setPosition(double positionMeters) {
        m_profiledPidController.setGoal(positionMeters);
        PIDEnabled = true;
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.positionMeters = m_encoder.getPosition();
        inputs.velocityMPS = m_encoder.getVelocity();

        if (PIDEnabled) {
            m_motor.setVoltage(
                m_profiledPidController.calculate(inputs.positionMeters) + m_feedforward.calculate(inputs.velocityMPS)
            );
        }

        inputs.appliedVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.currentAmps = m_motor.getOutputCurrent();

        Logger.recordOutput("Climber/PIDTargetRadians", m_profiledPidController.getGoal());
        Logger.recordOutput("Climber/PIDEnabled", PIDEnabled);
    }

    public void stop() {
        this.setVoltage(0.0);
    }
}