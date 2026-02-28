// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.hood;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;

public class HoodIONeo implements HoodIO {
    private final SparkMax m_leadMotor;

    private final SparkMaxConfig m_leadMotorConfig;
    private final RelativeEncoder m_alternateEncoder;

    private final SparkClosedLoopController m_closedLoopController;

    public HoodIONeo() {
        m_leadMotor = new SparkMax(HoodConstants.kCANId, MotorType.kBrushless);

        m_leadMotorConfig = new SparkMaxConfig();

        //create motor configs
        m_leadMotorConfig
            .inverted(HoodConstants.kMotorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) HoodConstants.kSmartCurrentLimit);

        m_leadMotorConfig.encoder
            .positionConversionFactor(HoodConstants.kPositionRelEncoderConversionFactor)
            .velocityConversionFactor(HoodConstants.kVelocityRelEncoderConversionFactor);

        m_leadMotorConfig.alternateEncoder
            .setSparkMaxDataPortConfig()
            .countsPerRevolution(8192)
            .inverted(true)
            .positionConversionFactor(HoodConstants.kPositionConversionFactor)
            .velocityConversionFactor(HoodConstants.kVelocityConversionFactor);

        m_leadMotorConfig.closedLoop
            .p(HoodConstants.kP)
            .i(HoodConstants.kI)
            .d(HoodConstants.kD)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        
        m_leadMotorConfig.closedLoop.feedForward
            .kS(HoodConstants.kS)
            .kV(HoodConstants.kV)
            .kA(HoodConstants.kA)
            .kCos(HoodConstants.kCos)
            .kCosRatio(HoodConstants.kCosRatio);

        m_leadMotorConfig.closedLoop.maxMotion
            .cruiseVelocity(HoodConstants.kCruiseVelocity)
            .maxAcceleration(HoodConstants.kMaxAccel)
            .allowedProfileError(0.01);

        m_leadMotorConfig.softLimit
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimitEnabled(false);

        m_leadMotor.configure(m_leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

        m_alternateEncoder = m_leadMotor.getEncoder();

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
    public void rezero() {
        m_alternateEncoder.setPosition(0);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.appliedVolts = m_leadMotor.getAppliedOutput() * m_leadMotor.getBusVoltage();
        inputs.currentAmps = m_leadMotor.getOutputCurrent();
        inputs.angleRadians = MathUtil.angleModulus(m_alternateEncoder.getPosition()); // TODO: confirm this
        inputs.velocityRadiansPerSecond = m_alternateEncoder.getVelocity();
        
        Logger.recordOutput("Hood/PIDTargetRadians", m_closedLoopController.getSetpoint());
    }
}