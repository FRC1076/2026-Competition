package frc.robot.subsystems.slapdown;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.SparkMax;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class SlapdownIONeo implements SlapdownIO {
    private final SparkMax m_motor;

    private final SparkMaxConfig m_motorConfig;
    private final RelativeEncoder m_relativeEncoder;

    private boolean PIDEnabled = false;
    private final ProfiledPIDController m_profiledPIDController;
    private final ArmFeedforward m_feedForwardController;

    public SlapdownIONeo() {
        m_motor = new SparkMax(SlapdownConstants.kCANId, MotorType.kBrushless);

        m_motorConfig = new SparkMaxConfig();

        // create motor configurations
        m_motorConfig
            .inverted(SlapdownConstants.kMotorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) SlapdownConstants.kSmartCurrentLimit);

        m_motorConfig.encoder
            .inverted(true)
            .positionConversionFactor(SlapdownConstants.kPositionConversionFactor)
            .velocityConversionFactor(SlapdownConstants.kVelocityConversionFactor);

        // configure motors
        m_motor.configure(m_motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        m_relativeEncoder = m_motor.getEncoder();

        m_profiledPIDController = new ProfiledPIDController(
            SlapdownConstants.kP,
            SlapdownConstants.kI,
            SlapdownConstants.kD,
            SlapdownConstants.kProfileConstraints
        );

        m_feedForwardController = new ArmFeedforward(
            SlapdownConstants.kS,
            SlapdownConstants.kG,
            SlapdownConstants.kV,
            SlapdownConstants.kA
        );
    }

    /**Sets slapdown motor's voltage */
    @Override
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
        PIDEnabled = false;
    }

    @Override
    public void periodic() {
        if (PIDEnabled){
            m_motor.setVoltage(
                m_profiledPIDController.calculate(m_relativeEncoder.getPosition()) +
                m_feedForwardController.calculate(m_profiledPIDController.getSetpoint().position, m_profiledPIDController.getSetpoint().velocity)
            );
        } 
    }

    @Override
    public void updateInputs(SlapdownIOInputs inputs) {
        inputs.appliedVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.currentAmps = m_motor.getOutputCurrent();

        inputs.angleRadians = m_relativeEncoder.getPosition();
        inputs.velocityRadiansPerSecond = m_relativeEncoder.getVelocity();

        Logger.recordOutput("Slapdown/PIDTargetRadians", m_profiledPIDController.getGoal());
        Logger.recordOutput("Slapdown/PIDEnabled", PIDEnabled);        
    }

    @Override 
    public void setPosition(double radians){
        m_profiledPIDController.setGoal(radians);
        PIDEnabled = true;
    }


}