// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.climb.climber;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimberIONeo implements ClimberIO {
    private final SparkMax m_motor;
    private final SparkMaxConfig m_motorConfig;
    private final SparkClosedLoopController m_closedLoopController;
    private final RelativeEncoder m_encoder;

    public ClimberIONeo() {
        m_motor = new SparkMax(ClimberConstants.kCANId, MotorType.kBrushless);
        m_motorConfig = new SparkMaxConfig();

        m_motorConfig
            .inverted(ClimberConstants.kMotorInverted)
            .smartCurrentLimit(ClimberConstants.kCurrentLimit)
            .voltageCompensation(ClimberConstants.kVoltageCompensation)
            .idleMode(ClimberConstants.kIdleMode);
        
        m_motorConfig.encoder
            .positionConversionFactor(ClimberConstants.kPositionConversionFactor)
            .velocityConversionFactor(ClimberConstants.kVelocityConversionFactor);

        m_motorConfig.closedLoop
            .p(ClimberConstants.kP)
            .i(ClimberConstants.kI)
            .d(ClimberConstants.kD);
        
        m_motorConfig.closedLoop.feedForward
            .kS(ClimberConstants.kS)
            .kV(ClimberConstants.kV)
            .kA(ClimberConstants.kA)
            .kG(ClimberConstants.kG);

        m_motorConfig.closedLoop.maxMotion
            .cruiseVelocity(ClimberConstants.kProfileConstraints.maxVelocity)
            .maxAcceleration(ClimberConstants.kProfileConstraints.maxAcceleration);

        m_motorConfig.softLimit
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimitEnabled(false); 
        
        m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        m_encoder = m_motor.getEncoder();
        m_encoder.setPosition(0);

        m_closedLoopController = m_motor.getClosedLoopController();
    }

    /** Sets the voltage for the climber motor */
    @Override
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }

    /** Sets the goal position for climber's PID controller in meters */
    @Override
    public void setPosition(double positionMeters) {
        m_closedLoopController.setSetpoint(positionMeters, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.appliedVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.currentAmps = m_motor.getOutputCurrent();
        inputs.positionMeters = m_encoder.getPosition();
        inputs.velocityMPS = m_encoder.getVelocity();

        Logger.recordOutput("Climber/PIDTargetMeters", m_closedLoopController.getSetpoint());
    }
}