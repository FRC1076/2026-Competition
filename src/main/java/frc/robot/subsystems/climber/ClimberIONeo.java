package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.subsystems.climber.ClimberConstants.HookConstants;

public class ClimberIONeo implements ClimberIO {
    private final SparkMax m_motor;
    private final SparkMaxConfig m_motorConfig;

    private final SparkMax m_hookMotor;
    private final SparkMaxConfig m_hookMotorConfig;

    private final RelativeEncoder m_encoder;

    private final ProfiledPIDController m_profiledPidController;
    private final ElevatorFeedforward m_feedforward;
    private boolean PIDEnabled = false;

    private final SparkClosedLoopController m_hookPIDController;

    public ClimberIONeo() {
        m_motor = new SparkMax(ClimberConstants.kCANId, MotorType.kBrushless);
        m_motorConfig = new SparkMaxConfig();

        m_hookMotor = new SparkMax(HookConstants.kCANId, MotorType.kBrushless);
        m_hookMotorConfig = new SparkMaxConfig();

        m_motorConfig
            .inverted(ClimberConstants.kMotorInverted)
            .smartCurrentLimit(ClimberConstants.kCurrentLimit)
            .voltageCompensation(ClimberConstants.kVoltageCompensation);
        
        m_motorConfig.encoder
            .positionConversionFactor(ClimberConstants.kPositionConversionFactor)
            .velocityConversionFactor(ClimberConstants.kVelocityConversionFactor);
        
        m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        m_encoder = m_motor.getEncoder();

        m_encoder.setPosition(0);

        m_hookMotorConfig
            .inverted(HookConstants.kMotorInverted)
            .smartCurrentLimit(HookConstants.kCurrentLimit)
            .voltageCompensation(HookConstants.kVoltageCompensation);

        m_hookMotorConfig.encoder
            .positionConversionFactor(HookConstants.kPositionConversionFactor)
            .velocityConversionFactor(HookConstants.kVelocityConversionFactor);

        m_hookMotorConfig.closedLoop
            .p(HookConstants.kP)
            .i(HookConstants.kI)
            .d(HookConstants.kD);

        m_hookMotor.configure(m_hookMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

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

        m_hookPIDController = m_hookMotor.getClosedLoopController();
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
    public void setHookVoltage(double volts) {
        m_hookMotor.setVoltage(volts);
    }

    @Override
    public void setHookPosition(double positionRadians) {
        m_hookPIDController.setSetpoint(positionRadians, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        if (PIDEnabled) {
            m_motor.setVoltage(
                m_profiledPidController.calculate(m_encoder.getPosition())+ m_feedforward.calculate(m_profiledPidController.getSetpoint().velocity)
            );
        }
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.appliedVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.currentAmps = m_motor.getOutputCurrent();
        inputs.positionMeters = m_encoder.getPosition();
        inputs.velocityMPS = m_encoder.getVelocity();

        Logger.recordOutput("Climber/PIDTargetRadians", m_profiledPidController.getGoal());
        Logger.recordOutput("Climber/PIDEnabled", PIDEnabled);
    }
}