package frc.robot.subsystems.hood;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
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
            .velocityConversionFactor(HoodConstants.kVelocityConversionFactor)
            .zeroOffset(HoodConstants.kZeroOffsetRadians / (2*Math.PI));

        m_leadMotor.configure(m_leadMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        m_absoluteEncoder = m_leadMotor.getAbsoluteEncoder();

        m_pidController = new ProfiledPIDController(
            HoodConstants.kP,
            HoodConstants.kI,
            HoodConstants.kD, 
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

    /** sets Hood's motor voltage */
    @Override
    public void setVoltage(double volts) {
        pidEnabled = false;
        m_leadMotor.setVoltage(volts);
    }

    /**Sets the setpoint of Hood PID Controller*/
    @Override
    public void setPosition(double radians) {
        pidEnabled = true;
        m_pidController.setGoal(radians);
    }

    @Override
    public void periodic() {
        if (pidEnabled) {
            m_leadMotor.setVoltage(
                m_pidController.calculate(m_absoluteEncoder.getPosition()) + m_feedforward.calculate(m_pidController.getSetpoint().position, m_pidController.getSetpoint().velocity)
            );
        }
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.appliedVolts = m_leadMotor.getAppliedOutput() * m_leadMotor.getBusVoltage();
        inputs.currentAmps = m_leadMotor.getOutputCurrent();
        inputs.angleRadians = MathUtil.angleModulus(m_absoluteEncoder.getPosition()); // TODO: confirm this
        inputs.velocityRadiansPerSecond = m_absoluteEncoder.getVelocity();
        
        Logger.recordOutput("Hood/PIDTargetRadians", m_pidController.getSetpoint());
        Logger.recordOutput("Hood/PIDEnabled", pidEnabled);
    }
}