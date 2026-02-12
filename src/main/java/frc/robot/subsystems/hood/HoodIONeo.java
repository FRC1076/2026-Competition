package frc.robot.subsystems.hood;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;

public class HoodIONeo implements HoodIO {
    private final SparkMax m_leadMotor;

    private final SparkMaxConfig m_leadMotorConfig;
    private final SparkAbsoluteEncoder m_absoluteEncoder;

    private final SparkClosedLoopController m_closedLoopController;

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

        m_leadMotorConfig.closedLoop
            .p(HoodConstants.kP)
            .i(HoodConstants.kI)
            .d(HoodConstants.kD);
        
        m_leadMotorConfig.closedLoop.feedForward
            .kS(HoodConstants.kS)
            .kV(HoodConstants.kV)
            .kA(HoodConstants.kA)
            .kCos(HoodConstants.kCos)
            .kCosRatio(HoodConstants.kCosRatio);

        m_leadMotor.configure(m_leadMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        m_absoluteEncoder = m_leadMotor.getAbsoluteEncoder();

        m_closedLoopController = m_leadMotor.getClosedLoopController();
    }

    /** sets Hood's motor voltage */
    @Override
    public void setVoltage(double volts) {
        m_leadMotor.setVoltage(volts);
    }

    /** Sets the setpoint of Hood PID Controller */
    @Override
    public void setPosition(double radians) {
        m_closedLoopController.setSetpoint(radians, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.appliedVolts = m_leadMotor.getAppliedOutput() * m_leadMotor.getBusVoltage();
        inputs.currentAmps = m_leadMotor.getOutputCurrent();
        inputs.angleRadians = MathUtil.angleModulus(m_absoluteEncoder.getPosition()); // TODO: confirm this
        inputs.velocityRadiansPerSecond = m_absoluteEncoder.getVelocity();
        
        Logger.recordOutput("Hood/PIDTargetRadians", m_closedLoopController.getSetpoint());
    }
}