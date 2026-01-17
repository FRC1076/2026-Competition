package frc.robot.subsystems.hood;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class HoodIONeo implements HoodIO {
    private final SparkMax m_leadMotor;

    private final SparkMaxConfig m_leadMotorConfig;
    private final SparkAbsoluteEncoder m_absoluteEncoder;

    private final ProfiledPIDController m_pidController;
    private final ArmFeedforward m_feedforward;
    private boolean pidEnabled = false;

    public HoodIONeo() {
        m_leadMotor = new SparkMax(HoodConstants.kCANId, MotorType.kBrushless);

        m_leadMotorConfig = new SparkMaxConfig();

        //create motor configs
        m_leadMotorConfig
            .inverted(HoodConstants.kMotorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) HoodConstants.kSmartCurrentLimit);

        m_leadMotorConfig.absoluteEncoder
            .setSparkMaxDataPortConfig()
            .inverted(true)
            .positionConversionFactor(HoodConstants.kPositionConversionFactor)
            .velocityConversionFactor(HoodConstants.kVelocityConversionFactor);

        m_leadMotorConfig.encoder 
            .positionConversionFactor(HoodConstants.kPositionConversionFactor)
            .velocityConversionFactor(HoodConstants.kVelocityConversionFactor);

        m_leadMotor.configure(m_leadMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        m_absoluteEncoder = m_leadMotor.getAbsoluteEncoder();

        m_pidController = new ProfiledPIDController(
            HoodConstants.kP,
            HoodConstants.kI,
            HoodConstants.kV, 
            new Constraints(
                HoodConstants.kMaxVelocityRadPerSec,
                HoodConstants.kMaxAccelerationRadPerSec2
            )
        );

        m_feedforward = new ArmFeedforward(
            HoodConstants.kS,
            HoodConstants.kG, 
            HoodConstants.kV, 
            HoodConstants.kA
        );
    }

    @Override
    public void setVoltage(double volts) {
        m_leadMotor.setVoltage(volts);
    }

    @Override
    public void setPosition(double radians) {
        m_pidController.setGoal(radians);
    }

    @Override
    public void updateInputs (HoodIOInputs inputs) {
        inputs.angleRadians = ((m_absoluteEncoder.getPosition()  - HoodConstants.kZeroOffsetRadians + Math.PI) % (2 * Math.PI) - Math.PI);
        inputs.velocityRadiansPerSecond = m_absoluteEncoder.getVelocity();

        if (pidEnabled) {
            setVoltage(
                m_pidController.calculate(inputs.angleRadians) + m_feedforward.calculate(inputs.angleRadians, inputs.velocityRadiansPerSecond)
            );
        }

        inputs.appliedVolts = m_leadMotor.getAppliedOutput() * m_leadMotor.getBusVoltage();
        inputs.leadCurrentAmps = m_leadMotor.getOutputCurrent();
        Logger.recordOutput("Hood/PIDTargetRadians", m_pidController.getSetpoint());
        Logger.recordOutput("Hood/PIDEnabled", pidEnabled);
    }
}